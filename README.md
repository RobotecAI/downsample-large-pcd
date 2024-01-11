# Downsample Large PCD

`Downsample Large PCD` is a tool for downsampling large point cloud using [pcl::VoxelGrid](http://pointclouds.org/documentation/classpcl_1_1_voxel_grid.html). It slices the point cloud into smaller parts appropriately (to ensure proper filtering), downsamples them, and merges downsampled slices into one point cloud.

The tool has been created to downsample large point clouds with any leaf size. `pcl::VoxelGrid` has a limitation for the number of voxels which refuses to downsample large point clouds for too small leaf size.

## Building on Ubuntu 22

1. Install required PCL libraries:
   ```bash
   apt install libpcl-dev
   ```
2. Build the project:
   ```bash
   mkdir build
   cd build
   cmake ..
   make -j
   ```

## Usage

 ```bash
./DownsampleLargePCD -in input.pcd -out output.pcd -leaf x,y,z -binary 1
```

where:
- `-in input.pcd` - path to the input pcd
- `-out output.pcd` - path for the output pcd
- `-leaf x,y,z` - the VoxelGrid leaf size (e.g. `-leaf 0.2,0.2,0.2`)
- `-binary 1` - (optional) 1 for binary output, 0 (default) for ASCII
