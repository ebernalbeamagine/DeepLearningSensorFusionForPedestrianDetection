# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.2.0] - 2021-08-31

### Added

### Changed
- Removed publishing of empty TargetLists (results in dead topic when no targets are detected)

### Removed
- MSE support (refer to MSE extension for this funtionality)

## [1.1.0] - 2020-04-07
### Added
- Support for UMRR11 T132 MSE (including launch files, config and converter node)

### Changed
- published and subscribed topics inside toolchain

### Removed
- delted tf functionality of spherical converter


## [0.1.1] - 2019-09-13 
### Added
- Support for CAN Messages with multiple headers
- Services for parameter request and command
- launch file for radar pipeline (useful when replaying .bag files)

### Changed
- node umrr_driver is now marked as required in launch file
  - whole sensor pipeline will exit when node is shutting down
- adapted error handling in umrr_driver node  

### Removed
- discrimination between wrong can socket and/or can spec, now one error

### Fixed
- handling of sensor.yaml: relative paths are now handled correctly

## [0.1.0] - 2019-03-11 
### Added
- node for filtering radar targets (pc2_filter.py) with dynamic reconfigure
- possibility to setup the sensor upon startup (center frequency, antenna mode)

### Changed
- automotive_radar.launch now reads in a .yaml file for configuration
- when receiving incomplete data from can whole list was dumped. Node now differs errors and dismisses
only the single target where information is missing


### Removed
- map_targets_2_ground_plane as this node is not needed for automotive sensors
- removed obsolete import statements

### Fixed
- in umrr_can_publisher node: fixed error handling when provided with wrong can spec and/or can socket. Code now differs between these events.

## [0.0.2] - 2019-02-01 
### Added
- Support for different CAN-Hardware.
It is now possible to specify a socket the Node should listen to. Parameter can be set in User Terminal when calling the launch file
- Sensor Status/Parameter send

### Changed
- ported functionality into their on classes

### Removed


### Fixed
- some bugs causing the node to crash

## [0.0.1a] - 2018-09-24
### Added
- Added some versioning information to readme as well as some appliance notes and necessary preliminaries

### Changed
- Readme contains now all necessary steps to run the node
- Naming of can spec. It is now automotive_spec.json
- Naming of launch file as it was misleading

### Removed
- Some links in readme.md that refer to tutorials that can not be reached froum outside smartmicro

### Fixed

## [0.0.1] - 2018-09-19
### Added
- Initial release

### Changed

### Removed

### Fixed



