// Tier: V
// Writes the two files the Conditions screen hands to run_campaign, with typed values and one edited limit, so a CI step can check
// that the Rust parser accepts them (the Rust parser is the authority on what is valid).
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/ui_files_writer.cpp -o ui_files_writer
//   ./ui_files_writer <campaign/test_definitions.json> <out conditions.json> <out definitions.json>
#include "DroneConditions.h"

#include <cstdio>
#include <fstream>
#include <sstream>

using namespace DroneApp;

int main(int argc, char** argv)
{
    if (argc < 4)
    {
        std::printf("usage: ui_files_writer <definitions.json> <out conditions.json> <out definitions.json>\n");
        return 2;
    }
    std::ifstream in(argv[1], std::ios::binary);
    std::stringstream ss;
    ss << in.rdbuf();
    ConditionsText T;
    T.Name = "ui_check";
    T.At("ambient_c") = "25";
    T.At("wind_speed_mps") = "3";
    T.At("payload_kg") = "0.25";
    T.At("payload_x_m") = "0.05";
    T.At("battery_soc") = "0.8";
    T.At("battery_temp_c") = "28";
    ConditionsValid V;
    std::string Err;
    if (!ValidateConditions(T, V, Err)) { std::printf("conditions refused: %s\n", Err.c_str()); return 1; }
    std::vector<EditableLimit> Climb = EditableLimits(Json::Parse(ss.str()).value, "climb");
    for (auto& E : Climb)
        if (E.Key == "target_height_m") E.Value = 30.0;
    std::string Defs;
    bool Edited = false;
    if (!WriteEditedDefinitions(ss.str(), Climb, Defs, Edited, Err) || !Edited) { std::printf("definitions refused: %s\n", Err.c_str()); return 1; }
    std::ofstream(argv[2], std::ios::binary) << WriteConditionsJson(V);
    std::ofstream(argv[3], std::ios::binary) << Defs;
    return 0;
}
