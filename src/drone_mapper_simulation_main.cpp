#include <drone_mapper/CompositionParser.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

// drone_mapper_simulation [<simulation.yaml>] [<output_path>]
//   - missing first arg  -> "simulation.yaml" in the cwd
//   - relative path       -> relative to the cwd
//   - absolute path       -> used as-is
//   - missing output path -> the cwd
int main(int argc, char** argv) {
    try {
        const std::optional<std::string> composition_arg =
            (argc >= 2) ? std::optional<std::string>{argv[1]} : std::nullopt;
        const std::filesystem::path composition_file = drone_mapper::resolveCompositionPath(composition_arg);
        const std::filesystem::path output_path =
            (argc >= 3) ? std::filesystem::path{argv[2]} : std::filesystem::current_path();

        drone_mapper::types::SimulationCompositionData composition =
            drone_mapper::parseCompositionYaml(composition_file);

        auto run_factory = std::make_unique<drone_mapper::SimulationRunFactoryImpl>();
        drone_mapper::SimulationManager manager{std::move(run_factory)};
        const drone_mapper::types::SimulationManagerReport report = manager.run(composition, output_path);

        std::cout << "ran " << report.runs.size() << " simulation run(s); wrote "
                  << (output_path / "simulation_output.yaml").string() << " and "
                  << (output_path / "output_results").string() << "/\n";
        return 0;
    } catch (const std::exception& e) {
        // Never crash: recover by reporting the failure and returning through main.
        std::cerr << "fatal: " << e.what() << '\n';
        return 1;
    }
}
