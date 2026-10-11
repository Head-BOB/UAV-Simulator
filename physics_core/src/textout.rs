// Tier: F
//! Small text helpers shared by the offline report writers (campaign): JSON string / number formatting and CSV fields.
//! Numbers are rounded to 12 significant digits so committed output files do not depend on the last bits of a value.

/// JSON string literal with escapes for quote, backslash and control characters.
#[must_use]
pub fn json_str(s: &str) -> String {
    use std::fmt::Write;
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            c if (c as u32) < 0x20 => {
                // writing to a String cannot fail
                write!(out, "\\u{:04x}", c as u32).ok();
            }
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

/// 12 significant digits: the last bits of a value are not information.
#[must_use]
pub fn round12(v: f64) -> f64 {
    format!("{v:.11e}").parse().unwrap_or(v)
}

/// A finite number as JSON / CSV text, `None` for NaN or infinity.
#[must_use]
pub fn num(v: f64) -> Option<String> {
    v.is_finite().then(|| format!("{}", round12(v)))
}

/// An optional number as JSON (`null` when absent), `None` if present but not finite.
#[must_use]
pub fn json_opt(v: Option<f64>) -> Option<String> {
    v.map_or(Some("null".to_string()), num)
}

/// A CSV field, quoted when it holds a comma, quote or newline.
#[must_use]
pub fn csv_field(s: &str) -> String {
    if s.contains([',', '"', '\n']) {
        format!("\"{}\"", s.replace('"', "\"\""))
    } else {
        s.to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_textout_json_string_escapes() {
        assert_eq!(json_str("a\"b\\c\nd\u{1}"), "\"a\\\"b\\\\c\\nd\\u0001\"");
        assert_eq!(json_str(""), "\"\"");
    }

    #[test]
    fn test_textout_numbers_are_12_digits_and_finite_only() {
        assert_eq!(num(0.1 + 0.2).as_deref(), Some("0.3"));
        assert_eq!(num(1.0 / 3.0).as_deref(), Some("0.333333333333"));
        assert_eq!(num(-2.5).as_deref(), Some("-2.5"));
        assert_eq!(num(0.0).as_deref(), Some("0"));
        assert_eq!(num(f64::NAN), None);
        assert_eq!(num(f64::INFINITY), None);
        assert_eq!(json_opt(None).as_deref(), Some("null"));
        assert_eq!(json_opt(Some(f64::NAN)), None);
        assert_eq!(json_opt(Some(1.5)).as_deref(), Some("1.5"));
    }

    #[test]
    fn test_textout_csv_quoting() {
        assert_eq!(csv_field("plain"), "plain");
        assert_eq!(csv_field("a,b"), "\"a,b\"");
        assert_eq!(csv_field("say \"hi\""), "\"say \"\"hi\"\"\"");
    }
}
