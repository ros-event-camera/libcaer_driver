^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package libcaer_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.5.4 (2026-04-26)
------------------
* fix rolling errors
* updated license string in package.xml
* warn of upgrade to firmware 1.0
* updated README and github workflow
* formatting fixes
* Cmake extension to support clang compiler and lsp
* fix doubled time in the imu (and frame) time_stamp
  Instead of passing current rostime to the frame and imu Callbacks,
  now correctly pass rosBaseTime\_ (rostime at startup) to both.
  Then the sensor_time of the SDK_packet is added, resulting in the time visible
  in the msgs.
  note: the change to the frame callback is technically not tested, because
  the hardware accessible to me does not support frame capture.
  Signed-of-by: Erik Schlenzka
* Contributors: Bernd Pfrommer, EschronS

1.5.3 (2025-09-01)
------------------
* support new image transport node interface
* Contributors: Bernd Pfrommer

1.5.2 (2025-07-29)
------------------
* support new transport api
* Contributors: Bernd Pfrommer

1.5.1 (2025-05-22)
------------------
* stop using ament_target_dependencies
* Contributors: Bernd Pfrommer

1.5.0 (2025-04-10)
------------------
* 16 bit APS image and color cam support
* support for recording under Jazzy+
* updated installation instructions
* Contributors: Bernd Pfrommer

1.0.3 (2024-05-30)
------------------
* remove repos file, build only on recent distros
* use libcaer_vendor
* initial release on rolling?
* Contributors: Bernd Pfrommer, Thies Lennart Alff
