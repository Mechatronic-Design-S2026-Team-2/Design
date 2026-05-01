Precomputed 17-bit Klann lookup tables belong in this directory.

Generate them on a desktop or other fast machine before deployment on the Jetson:

```bash
ros2 run hexapod_hardware_cpp klann_lut_generator \
  /path/to/linkage_measured.yaml \
  /path/to/output_dir \
  131072
```

That writes one `.klut` file per leg plus `linkage_precomputed.yaml`.
Point `linkage_yaml` at that generated YAML for deployment.
