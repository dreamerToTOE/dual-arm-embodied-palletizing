# Evidence index

- `occupancy_seed_28`, `occupancy_seed_29`, `occupancy_seed_30`: final candidate experiments. Occupied cubes participate in precontact planning and in later candidate validation.
- `timing_methods`: independent reruns of the three methods under the same eight-worker configuration.
- `negative_control`: object/obstacle collision and protected object-ID rejection, plus a nominal prior-verifier replay.
- `reference`: frozen robot URDF/SRDF and SHA-256 hashes of installed FR3 collision meshes.
- `study_20261104`: **invalid** exploratory run; collision meshes were missing. Kept only for audit.
- `study_complete_mesh`, `seed_28`, `seed_29`, `seed_30`, and `final_seed_28`, `final_seed_29`, `final_seed_30`: earlier complete-mesh intermediate runs that revalidated an empty-scene approach against occupancy, but did not plan the approach with occupancy. They are retained for audit and are not the final benchmark.
