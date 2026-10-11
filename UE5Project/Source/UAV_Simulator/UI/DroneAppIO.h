// Tier: V
// Engine side of DroneApp::AppIO: files through the engine's file helpers, helper programs as child processes with their output captured
// to a log file. NOT COMPILED OR RUN by its author: TODO(HUMAN-VERIFY: API names on the project's Unreal version).
#pragma once

#include "CoreMinimal.h"
#include "HAL/PlatformProcess.h"

#include "DroneAppModel.h"
#include "DroneProgress.h"

class FDroneAppIO : public DroneApp::AppIO
{
public:
	explicit FDroneAppIO(const FString& InWorkingDirectory) : WorkingDirectory(InWorkingDirectory) {}
	virtual ~FDroneAppIO() override;

	virtual bool ReadText(const std::string& Path, std::string& Out) override;
	virtual bool WriteText(const std::string& Path, const std::string& Text) override;
	virtual bool Exists(const std::string& Path) override;
	virtual int StartProcess(const DroneApp::Command& C, const std::string& InLogPath) override;
	virtual bool PollProcess(int Handle, int& ExitCode) override;
	virtual void OpenPath(const std::string& Path) override;
	virtual bool ReadProgress(int Handle, DroneApp::ProgressSnapshot& Out) override;
	virtual void CancelProcess(int Handle) override;

private:
	struct FJob
	{
		FProcHandle Proc;
		void* ReadPipe = nullptr;
		void* WritePipe = nullptr;
		FString Log;
		FString LogPath;
		bool bDone = false;
		int32 ExitCode = -1;
		DroneApp::ProgressParser Parser;   // UAVPROGRESS lines, parsed as the output arrives
		double LastLogWriteS = 0.0;
	};
	TArray<FJob> Jobs;   // handle = index + 1
	FString WorkingDirectory;
	void CloseJob(FJob& J);
};
