// Tier: F
// Only the `onnx` FFI entry points use the registry; the pure logic stays compiled so tests and
// miri (no-default-features) still run it.
#![cfg_attr(not(feature = "onnx"), allow(dead_code))]

//! Registry of live surrogate handles and the per-handle locks (review finding B).
//!
//! Two lock levels, never held the other way round:
//! 1. `Registry` lock: held only for a map lookup / insert / remove. No foreign code (ONNX
//!    inference) ever runs under it, so one slow or panicking query cannot block or poison
//!    every other handle.
//! 2. `Slot` lock: one per handle, held while that handle is queried or freed. A caller clones
//!    the `Arc<Slot>` under the registry lock, drops the registry lock, then takes the slot
//!    lock. `alive` is checked UNDER the slot lock, so a handle freed after the lookup is seen
//!    as dead instead of being dereferenced.
//!
//! Poison policy: a poisoned registry lock is cleared and that one call reports
//! `RegistryError::Recovered` (FFI code 98: nothing was done, retry). A poisoned slot lock marks
//! the handle `faulted` for good (FFI code 99 on use); it can still be unloaded.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::{Arc, Mutex, MutexGuard};

/// Records a newly created handle address as live.
pub fn register_in(live: &mut BTreeSet<usize>, addr: usize) {
    live.insert(addr);
}

/// Removes a handle address. Returns false if it was not live (unknown or already unloaded).
pub fn unregister_from(live: &mut BTreeSet<usize>, addr: usize) -> bool {
    live.remove(&addr)
}

/// True when the step's two handle addresses are usable: 0 (null) is allowed, a non-null
/// address must be live, and the two must differ (the step takes `&mut` to both, so equal
/// non-null addresses would alias).
pub fn step_handles_ok(aero_addr: usize, fea_addr: usize, live: &BTreeSet<usize>) -> bool {
    let live_or_null = |a: usize| a == 0 || live.contains(&a);
    live_or_null(aero_addr) && live_or_null(fea_addr) && (aero_addr == 0 || aero_addr != fea_addr)
}

/// State guarded by a handle's own lock.
#[derive(Debug)]
pub struct SlotState {
    /// False once the handle has been (or is being) freed.
    pub alive: bool,
    /// True once a panic was observed while this handle's lock was held. Permanent.
    pub faulted: bool,
}

/// Per-handle lock. See the module docs.
#[derive(Debug)]
pub struct Slot {
    state: Mutex<SlotState>,
}

/// What a caller must do after locking a slot.
#[derive(Debug, PartialEq, Eq)]
pub enum SlotStatus {
    Usable,
    /// Freed (or being freed): FFI "not live" code.
    Dead,
    /// A panic happened while the handle was in use: FFI panic code.
    Faulted,
}

impl Slot {
    pub fn new() -> Self {
        Self {
            state: Mutex::new(SlotState {
                alive: true,
                faulted: false,
            }),
        }
    }

    /// Locks the slot. A poisoned lock is recovered and the handle is marked `faulted`.
    pub fn lock(&self) -> MutexGuard<'_, SlotState> {
        match self.state.lock() {
            Ok(g) => g,
            Err(poisoned) => {
                let mut g = poisoned.into_inner();
                g.faulted = true;
                self.state.clear_poison();
                g
            }
        }
    }
}

impl Default for Slot {
    fn default() -> Self {
        Self::new()
    }
}

/// Classifies a locked slot. `Dead` wins over `Faulted`: a freed handle is simply not live.
pub fn slot_status(state: &SlotState) -> SlotStatus {
    if !state.alive {
        SlotStatus::Dead
    } else if state.faulted {
        SlotStatus::Faulted
    } else {
        SlotStatus::Usable
    }
}

/// Locks up to two distinct slots in ascending address order, so two threads stepping with the
/// same two handles in opposite roles cannot deadlock. The caller guarantees the addresses
/// differ when both are present (`step_handles_ok`).
pub fn lock_pair<'a>(
    a: Option<(usize, &'a Slot)>,
    b: Option<(usize, &'a Slot)>,
) -> (
    Option<MutexGuard<'a, SlotState>>,
    Option<MutexGuard<'a, SlotState>>,
) {
    match (a, b) {
        (Some((addr_a, slot_a)), Some((addr_b, slot_b))) => {
            if addr_a <= addr_b {
                let guard_a = slot_a.lock();
                let guard_b = slot_b.lock();
                (Some(guard_a), Some(guard_b))
            } else {
                let guard_b = slot_b.lock();
                let guard_a = slot_a.lock();
                (Some(guard_a), Some(guard_b))
            }
        }
        (Some((_, slot_a)), None) => (Some(slot_a.lock()), None),
        (None, Some((_, slot_b))) => (None, Some(slot_b.lock())),
        (None, None) => (None, None),
    }
}

/// The registry lock was poisoned. It has been cleared; the call that saw this did nothing.
#[derive(Debug, PartialEq, Eq)]
pub enum RegistryError {
    Recovered,
}

#[derive(Debug)]
pub struct RegistryInner {
    live: BTreeSet<usize>,
    slots: BTreeMap<usize, Arc<Slot>>,
}

/// Slots for the (aero, fea) handles of one step; `None` for a null handle.
pub type StepSlots = (Option<Arc<Slot>>, Option<Arc<Slot>>);

#[derive(Debug)]
pub struct Registry {
    inner: Mutex<RegistryInner>,
}

impl Registry {
    pub const fn new() -> Self {
        Self {
            inner: Mutex::new(RegistryInner {
                live: BTreeSet::new(),
                slots: BTreeMap::new(),
            }),
        }
    }

    fn lock(&self) -> Result<MutexGuard<'_, RegistryInner>, RegistryError> {
        match self.inner.lock() {
            Ok(g) => Ok(g),
            Err(poisoned) => {
                drop(poisoned);
                self.inner.clear_poison();
                Err(RegistryError::Recovered)
            }
        }
    }

    /// Records `addr` as live with a fresh slot.
    pub fn register(&self, addr: usize) -> Result<(), RegistryError> {
        let mut g = self.lock()?;
        register_in(&mut g.live, addr);
        g.slots.insert(addr, Arc::new(Slot::new()));
        Ok(())
    }

    /// The slot of a live handle, or `None` if `addr` is not live.
    pub fn slot_of(&self, addr: usize) -> Result<Option<Arc<Slot>>, RegistryError> {
        let g = self.lock()?;
        Ok(g.slots.get(&addr).cloned())
    }

    /// Removes a handle from the registry and returns its slot (`None` if it was not live).
    /// The caller then locks the slot, which waits for any query still using the handle.
    pub fn remove(&self, addr: usize) -> Result<Option<Arc<Slot>>, RegistryError> {
        let mut g = self.lock()?;
        if unregister_from(&mut g.live, addr) {
            Ok(g.slots.remove(&addr))
        } else {
            Ok(None)
        }
    }

    /// Slots for one step. `Ok(None)` when `step_handles_ok` rejects the pair.
    pub fn step_slots(
        &self,
        aero_addr: usize,
        fea_addr: usize,
    ) -> Result<Option<StepSlots>, RegistryError> {
        let g = self.lock()?;
        if !step_handles_ok(aero_addr, fea_addr, &g.live) {
            return Ok(None);
        }
        Ok(Some((
            g.slots.get(&aero_addr).cloned(),
            g.slots.get(&fea_addr).cloned(),
        )))
    }
}

impl Default for Registry {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;
    use std::time::Duration;

    /// Poisons the registry lock by panicking on another thread while holding it.
    fn poison_for_test(r: &Arc<Registry>) {
        let me = Arc::clone(r);
        let joined = std::thread::spawn(move || {
            let _held = me.inner.lock();
            panic!("deliberate poison");
        })
        .join();
        assert!(joined.is_err());
    }

    // Hand calc: register 0x10; lookup finds it; 0x20 is unknown; remove returns the slot once,
    // then None (double unload); after removal a lookup finds nothing.
    #[test]
    fn test_registry_register_lookup_remove() {
        let r = Registry::new();
        r.register(0x10).expect("lock");
        assert!(r.slot_of(0x10).expect("lock").is_some());
        assert!(r.slot_of(0x20).expect("lock").is_none());
        assert!(r.remove(0x10).expect("lock").is_some());
        assert!(r.remove(0x10).expect("lock").is_none());
        assert!(r.slot_of(0x10).expect("lock").is_none());
    }

    // Hand calc, live = {0x10, 0x20}: (0,0) ok; (0x10,0) ok; (0x10,0x20) ok; (0x10,0x10) alias;
    // (0x30,0) unknown; (0,0x30) unknown.
    #[test]
    fn test_registry_step_slots_truth_table() {
        let r = Registry::new();
        r.register(0x10).expect("lock");
        r.register(0x20).expect("lock");
        let ok = |a, b| r.step_slots(a, b).expect("lock").is_some();
        assert!(ok(0, 0));
        assert!(ok(0x10, 0));
        assert!(ok(0, 0x20));
        assert!(ok(0x10, 0x20));
        assert!(!ok(0x10, 0x10));
        assert!(!ok(0x30, 0));
        assert!(!ok(0, 0x30));
        let (a, f) = r.step_slots(0x10, 0).expect("lock").expect("ok");
        assert!(a.is_some() && f.is_none());
    }

    // The design claim of finding B: the registry lock is NOT held while a handle is in use.
    // If the registry lock were held during "inference", the lookup below would block forever
    // (single thread, so a hang means the test fails by timeout).
    #[test]
    fn test_registry_not_held_while_slot_is_locked() {
        let r = Registry::new();
        r.register(0x10).expect("lock");
        r.register(0x20).expect("lock");
        let slot = r.slot_of(0x10).expect("lock").expect("live");
        let in_use = slot.lock();
        assert!(r.slot_of(0x20).expect("lock").is_some());
        assert!(r.register(0x30).is_ok());
        assert!(r.remove(0x20).expect("lock").is_some());
        drop(in_use);
    }

    // Unload must wait for a query that still uses the handle: while a "query" holds the slot,
    // retiring it blocks; after release it completes and the slot reads Dead.
    #[test]
    fn test_unload_waits_for_in_flight_query() {
        let slot = Arc::new(Slot::new());
        let in_use = slot.lock();
        let (tx, rx) = mpsc::channel();
        let other = Arc::clone(&slot);
        let worker = std::thread::spawn(move || {
            let mut g = other.lock();
            g.alive = false;
            tx.send(()).expect("send");
        });
        assert!(rx.recv_timeout(Duration::from_millis(100)).is_err());
        drop(in_use);
        rx.recv_timeout(Duration::from_secs(5)).expect("retired");
        worker.join().expect("join");
        assert_eq!(slot_status(&slot.lock()), SlotStatus::Dead);
    }

    // A query that looked the slot up before unload and locks it after must see Dead.
    #[test]
    fn test_stale_slot_after_unload_is_dead() {
        let r = Registry::new();
        r.register(0x10).expect("lock");
        let stale = r.slot_of(0x10).expect("lock").expect("live");
        let removed = r.remove(0x10).expect("lock").expect("was live");
        removed.lock().alive = false;
        assert_eq!(slot_status(&stale.lock()), SlotStatus::Dead);
    }

    // Poisoned registry: first call after the panic reports Recovered (nothing done), the
    // second works, and earlier entries survived. It must not report an error forever.
    #[test]
    fn test_registry_poison_is_recovered_once() {
        let r = Arc::new(Registry::new());
        r.register(0x10).expect("lock");
        poison_for_test(&r);
        assert_eq!(r.slot_of(0x10).err(), Some(RegistryError::Recovered));
        assert!(r.slot_of(0x10).expect("recovered").is_some());
        assert!(r.register(0x20).is_ok());
    }

    // Poisoned slot: the handle becomes Faulted permanently (not Usable), is still removable.
    #[test]
    fn test_slot_poison_marks_handle_faulted() {
        let slot = Arc::new(Slot::new());
        let other = Arc::clone(&slot);
        let joined = std::thread::spawn(move || {
            let _held = other.lock();
            panic!("deliberate poison");
        })
        .join();
        assert!(joined.is_err());
        assert_eq!(slot_status(&slot.lock()), SlotStatus::Faulted);
        assert_eq!(slot_status(&slot.lock()), SlotStatus::Faulted);
        slot.lock().alive = false;
        assert_eq!(slot_status(&slot.lock()), SlotStatus::Dead);
    }

    #[test]
    fn test_slot_status_truth_table() {
        let s = |alive, faulted| slot_status(&SlotState { alive, faulted });
        assert_eq!(s(true, false), SlotStatus::Usable);
        assert_eq!(s(true, true), SlotStatus::Faulted);
        assert_eq!(s(false, false), SlotStatus::Dead);
        assert_eq!(s(false, true), SlotStatus::Dead);
    }

    // Two threads lock the same two slots in opposite argument order 200000 times each. Without
    // the ascending-address order this deadlocks; finishing is the pass condition.
    #[test]
    fn test_lock_pair_opposite_orders_do_not_deadlock() {
        let a = Arc::new(Slot::new());
        let b = Arc::new(Slot::new());
        let (a1, b1, a2, b2) = (a.clone(), b.clone(), a.clone(), b.clone());
        let t1 = std::thread::spawn(move || {
            for _ in 0..200_000 {
                let _g = lock_pair(Some((1, &a1)), Some((2, &b1)));
            }
        });
        let t2 = std::thread::spawn(move || {
            for _ in 0..200_000 {
                let _g = lock_pair(Some((2, &b2)), Some((1, &a2)));
            }
        });
        t1.join().expect("t1");
        t2.join().expect("t2");
        assert!(lock_pair(None, None).0.is_none());
        assert!(lock_pair(Some((1, &a)), None).0.is_some());
        assert!(lock_pair(None, Some((2, &b))).1.is_some());
    }
}
