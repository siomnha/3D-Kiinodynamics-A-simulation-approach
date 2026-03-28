#include <octomap/OcTree.h>

#include <cstdlib>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    if (argc < 3 || argc > 4) {
        std::cerr << "usage: clean_bt input.bt output.bt [occupancy_threshold]\n";
        return 1;
    }

    const std::string input_path = argv[1];
    const std::string output_path = argv[2];
    const double threshold = (argc == 4) ? std::atof(argv[3]) : 0.7;

    octomap::OcTree tree(input_path);
    if (tree.size() == 0) {
        std::cerr << "failed to load or empty octomap: " << input_path << "\n";
        return 1;
    }

    std::cout << "resolution: " << tree.getResolution() << "\n";
    std::cout << "old occupancy threshold: " << tree.getOccupancyThres() << "\n";
    std::cout << "new occupancy threshold: " << threshold << "\n";

    tree.setOccupancyThres(threshold);
    tree.toMaxLikelihood();
    tree.prune();
    tree.updateInnerOccupancy();

    if (!tree.writeBinary(output_path)) {
        std::cerr << "failed to write output: " << output_path << "\n";
        return 1;
    }

    std::cout << "wrote " << output_path << "\n";
    return 0;
}
