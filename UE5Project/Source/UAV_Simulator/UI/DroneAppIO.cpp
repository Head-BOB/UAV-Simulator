// Tier: V
// NOT COMPILED OR RUN by its author. See DroneAppIO.h.
#include "DroneAppIO.h"

#include "HAL/FileManager.h"
#include "HAL/PlatformProcess.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

namespace
{
FString Utf8(const std::string& S) { return FString(UTF8_TO_TCHAR(S.c_str())); }
std::string ToStd(const FString& S) { return std::string(TCHAR_TO_UTF8(*S)); }
}  // namespace

FDroneAppIO::~FDroneAppIO()
{
	for (FJob& J : Jobs) CloseJob(J);
}

void FDroneAppIO::CloseJob(FJob& J)
{
	if (J.ReadPipe || J.WritePipe) FPlatformProcess::ClosePipe(J.ReadPipe, J.WritePipe);
	J.ReadPipe = nullptr;
	J.WritePipe = nullptr;
	if (J.Proc.IsValid()) FPlatformProcess::CloseProc(J.Proc);
}

bool FDroneAppIO::ReadText(const std::string& Path, std::string& Out)
{
	FString Text;
	if (!FFileHelper::LoadFileToString(Text, *Utf8(Path))) return false;
	Out = ToStd(Text);
	return true;
}

bool FDroneAppIO::WriteText(const std::string& Path, const std::string& Text)
{
	const FString P = Utf8(Path);
	IFileManager::Get().MakeDirectory(*FPaths::GetPath(P), true);
	return FFileHelper::SaveStringToFile(Utf8(Text), *P, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
}

bool FDroneAppIO::Exists(const std::string& Path)
{
	const FString P = Utf8(Path);
	return IFileManager::Get().FileExists(*P) || IFileManager::Get().DirectoryExists(*P);
}

int FDroneAppIO::StartProcess(const DroneApp::Command& C, const std::string& InLogPath)
{
	FJob J;
	J.LogPath = Utf8(InLogPath);
	if (!FPlatformProcess::CreatePipe(J.ReadPipe, J.WritePipe)) return -1;
	uint32 Pid = 0;
	// stdout and stderr of the child both go to the write end of the pipe. TODO(HUMAN-VERIFY): CreateProc argument order; stderr is
	// merged with stdout on Windows when only PipeWriteChild is given.
	J.Proc = FPlatformProcess::CreateProc(*Utf8(C.Program), *Utf8(C.ArgumentString()), false, true, true, &Pid, 0, WorkingDirectory.IsEmpty() ? nullptr : *WorkingDirectory, J.WritePipe, nullptr);
	if (!J.Proc.IsValid())
	{
		CloseJob(J);
		return -1;
	}
	Jobs.Add(J);
	return Jobs.Num();
}

bool FDroneAppIO::PollProcess(int Handle, int& ExitCode)
{
	if (Handle < 1 || Handle > Jobs.Num()) { ExitCode = -1; return true; }
	FJob& J = Jobs[Handle - 1];
	if (J.bDone) { ExitCode = J.ExitCode; return true; }
	J.Log += FPlatformProcess::ReadPipe(J.ReadPipe);
	if (FPlatformProcess::IsProcRunning(J.Proc)) return false;
	J.Log += FPlatformProcess::ReadPipe(J.ReadPipe);   // whatever was written just before exit
	int32 Code = -1;
	FPlatformProcess::GetProcReturnCode(J.Proc, &Code);
	J.ExitCode = Code;
	J.bDone = true;
	CloseJob(J);
	WriteText(ToStd(J.LogPath), ToStd(J.Log));
	ExitCode = Code;
	return true;
}

void FDroneAppIO::OpenPath(const std::string& Path)
{
	FPlatformProcess::LaunchFileInDefaultExternalApplication(*Utf8(Path));
}
