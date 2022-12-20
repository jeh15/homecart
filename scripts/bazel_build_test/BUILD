# BUILD

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "demo",
    srcs = ["demo.cc",],
    linkstatic = True,
    deps = [
        "@ur_rtde",
        "@drake//lcmtypes:schunk",
        "@gflags//:gflags",
        "@lcm//:lcm",
        ],
    linkopts = [
        "-lpthread",
        ],
)