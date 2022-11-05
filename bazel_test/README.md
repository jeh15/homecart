# Questions:
`cmake()` bazel rule:

Initially did not provide an output so defaults to `out_static_libs` and did not know names so threw error `CMake: Building ur_rtde failed: not all outputs were created or valid`.

Changed `out_headers_only` and resulted in successful build of `ur_rtde` but did not have the implementation for the header resulting in undefined reference.

Tried to scour `ur_rtde` CMake files to see if it makes a static or shared lib but could not find anything.

Noticed in `bazel-bin/src` a directory called `copy_ur_rtde` where in `lib` it had shared lib files.

Added the following:
```python
out_shared_libs = [
        "librtde.so",
        "librtde.so.1.5",
        "librtde.so.1.5.5",
        ],
```

* How do I figure out that CMake was producing these shared lib files with these specific names?
* Can I get bazel to figure this out on its own?
* How can I better structure this repo with bazel?