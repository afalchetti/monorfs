# MonoRFS

Visual SLAM system using Random Finite Sets.

Version: 0.8.

## Description

This is a simple program that implements a Rao-Blackwellized PHD filter and
a Loopy PHD offline processor for autonomous navigation in 1D/2D/3D space.
It consists of several components:

* A SLAM solver unit, which is the responsible of handling the Bayes updates
  over the map and pose state of a vehicle.
  
* A simulator, which gives live feedback to the user about the state of the
  system and allows him to control the autonomous device.
  
* A viewer, that can reproduce previously recorded runs of the simulator, to
  present to others or for fine-grain analysis.
  
* A device interface, which can talk to a RGBD camera and process its output
  images (both color and depth) and generate the necessary data for the SLAM
  solver to work on.
  
* A device faker unit, which takes PNG streams and creates oni files suitable as
  input to the device interface, allowing video reuse and easy reproducibility.
  
* A post-processing script that generates useful plots about the simulated run.

* An alternative SLAM solver, which uses the iSAM2 algorithm to estimate the
  world state. It uses the gtsam library.

Its main purpose is to explore new research ideas, not necessarily work in
production.

## Author

[Angelo Falchetti](https://github.com/afalchetti)

## Usage examples

Start analyzing a live RGBD stream with assumed odometry "movroom.in",
200 particles and solving both mapping and localization

```
monorfs -i=kinect -f=room.oni -c=movroom.in -p=200
```

Same as above but saving the output to a specified record file
(default = "data.zip")

```
monorfs -i=kinect -f=room.oni -c=movroom.in -p=200 -r=output.zip
```

Start analyzing a recorded RGBD stream from file "room.oni",
40 particles and perfect localization

```
monorfs -i=kinect -f=room.oni -c=movroom.in -p=40 -y
```

Start a new simulation with a world description given in file "map.world",
movement commands in "moves.in" (odometry), with 20 particles and
assuming perfect localization (at startup, this can be changed later)

```
monorfs -i=simulation -f=map.world -c=movements.in -p=20 -y
```

Similar to above but use isam2 SLAM algorithm

```
monorfs -i=simulation -f=map.world -c=movements.in -a=isam2
```

The same as the above, but in headless mode

```
monorfs -i=simulation -f=map.world -c=movements.in -a=isam2 -x
```

Using a configuration file to change the algorithm's parameters

```
monorfs -i=simulation -f=map.world -c=movements.in -p=200 -g=config.cfg
```

Use a prerecorded data file

```
monorfs -i=record -f=data.zip -c=movements.in
```

Open a previous run with a Viewer

```
monorfs -v -r=recording.zip
```

Use filtering mode in the viewer, i.e.
don't update past history

```
monorfs -v -r=recording.zip -H=filter
```

## License

This project is under the New BSD license. See [LICENSE](LICENSE).
