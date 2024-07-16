# euclidean_motion

```bash
mkdir build
cd build

# had some issues with infinite loop during tests preparation.
cmake -DCMAKE_CXX_COMPILER=/usr/bin/clang++ ..
make

# run main.
./main ../test/data/src.txt ../test/data/dst.txt

# run tests.
./test/tests --gtest_filter=...
```

## Algo

Key ideas:

- shift points' center of mass to (0, 0)
- sort points by radius
- group points by eps-rings they are lying in
- check cyclic shifts transforming "source ring" to "destination ring"
- such common for every "ring" shift is a result rotation
- calc transformation using rotation and centers of mass
