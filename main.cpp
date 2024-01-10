#include <filesystem>

#include <fmt/core.h>

#include <pcl/common/common.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

namespace fs = std::filesystem;

// Based on: https://github.com/PointCloudLibrary/pcl/blob/master/filters/include/pcl/filters/impl/voxel_grid.hpp#L253
constexpr size_t PclVoxelsLimit = std::numeric_limits<std::int32_t>::max();

void printHelp()
{
	fmt::print("usage: ./DownsampleLargePCD -in input.pcd -out output.pcd -leaf x,y,z\n");
	fmt::print("  where:\n");
	fmt::print("      -in input.pcd   = input pcd for downsampling\n");
	fmt::print("      -out output.pcd = output pcd\n");
	fmt::print("      -leaf x,y,z     = the VoxelGrid leaf size\n");
}

struct VoxelAxisHelper
{
	std::string axis;
	float minRange;
	float maxRange;
	float leaf;

	size_t getVoxelCount() const { return static_cast<size_t>((maxRange - minRange) / leaf) + 1; }
};

int main(int argc, char** argv)
{
	fmt::print("Downsample large point cloud using pcl::VoxelGrid. For more information, use: {} -h\n", argv[0]);

	// Parse arguments
	bool showHelp = false;
	std::string inPcd;
	std::string outPcd;
	std::array<float, 3> leaf = {0.0f, 0.0f, 0.0f};
	pcl::console::parse_argument(argc, argv, "-h", showHelp);
	if (showHelp) {
		printHelp();
		return 0;
	}
	if (pcl::console::parse_argument(argc, argv, "-in", inPcd) == -1) {
		fmt::print(stderr, "Argument `-in` is required\n");
		return -1;
	}
	if (pcl::console::parse_argument(argc, argv, "-out", outPcd) == -1) {
		fmt::print(stderr, "Argument `-out` is required\n");
		return -1;
	}
	if (pcl::console::parse_3x_arguments(argc, argv, "-leaf", leaf[0], leaf[1], leaf[2]) == -1) {
		fmt::print(stderr, "Argument `-leaf` is required (e.g., -leaf 0.2,0.2,0.2)\n");
		return -1;
	}
	if (!(leaf[0] > 0.0f && leaf[1] > 0.0f && leaf[2] > 0.0f)) {
		fmt::print(stderr, "Leaf size must be positive. Got: {}, {}, {}\n", leaf[0], leaf[1], leaf[2]);
		return -1;
	}

	fmt::print("Loading input point cloud...\n");

	pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(inPcd, *inCloud) != 0) {
		// Error message already printed by loadPCDFile function
		return -1;
	}

	fmt::print("Estimating slicing parameters...\n");

	// Find point cloud slicing parameters: along which axis, range and step
	pcl::PointXYZ minPoint, maxPoint;
	pcl::getMinMax3D(*inCloud, minPoint, maxPoint);

	std::array<VoxelAxisHelper, 3> voxelAxes = {
	    VoxelAxisHelper{
	                    .axis = "x",
	                    .minRange = minPoint.x,
	                    .maxRange = maxPoint.x,
	                    .leaf = leaf[0],
	                    },
	    VoxelAxisHelper{
	                    .axis = "y",
	                    .minRange = minPoint.y,
	                    .maxRange = maxPoint.y,
	                    .leaf = leaf[1],
	                    },
	    VoxelAxisHelper{
	                    .axis = "z",
	                    .minRange = minPoint.z,
	                    .maxRange = maxPoint.z,
	                    .leaf = leaf[2],
	                    }
    };

	// Axis with the most voxel count
	VoxelAxisHelper selectedVoxelAxis = *std::max_element(voxelAxes.begin(), voxelAxes.end(),
	                                                      [](auto a, auto b) { return a.getVoxelCount() < b.getVoxelCount(); });

	size_t voxelCountForOneSlice = (voxelAxes[0].getVoxelCount() * voxelAxes[1].getVoxelCount() *
	                                voxelAxes[2].getVoxelCount()) /
	                               selectedVoxelAxis.getVoxelCount();
	size_t sliceCountNotOverflowing = PclVoxelsLimit / voxelCountForOneSlice;

	float currentRangeMin = selectedVoxelAxis.minRange;
	float rangeStep = static_cast<float>(sliceCountNotOverflowing) * selectedVoxelAxis.leaf;
	size_t slicesCount = std::ceil((selectedVoxelAxis.maxRange - selectedVoxelAxis.minRange) / rangeStep);
	int currentSlice = 1;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> downsampledSlices;
	while (currentRangeMin <= selectedVoxelAxis.maxRange) {
		fmt::print("Downsampling point cloud slice: {}/{}\n", currentSlice, slicesCount);
		pcl::PointCloud<pcl::PointXYZ>::Ptr currentCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

		pcl::VoxelGrid<pcl::PointXYZ> filter;
		filter.setInputCloud(inCloud);
		filter.setFilterFieldName(selectedVoxelAxis.axis);
		filter.setFilterLimits(currentRangeMin, currentRangeMin + rangeStep);
		filter.setLeafSize(leaf[0], leaf[1], leaf[2]);
		filter.filter(*currentCloud);
		downsampledSlices.push_back(currentCloud);

		++currentSlice;
		currentRangeMin += rangeStep;
	}

	fmt::print("Concatenating downsampled slices...\n");
	pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	for (auto&& pc : downsampledSlices) {
		*outCloud += *pc;
	}

	fmt::print("Saving to the file...\n");
	pcl::io::savePCDFile(outPcd, *outCloud, false);
	fmt::print("Downsampled point cloud has been saved to `{}`\n", outPcd);

	return 0;
}
