# BUILD
cc_binary(
    name = "gripper_close",
    srcs = ["gripper_close.cc",],
    linkstatic = True,
    linkopts = ["-lpthread"],
    deps = [
        "@ur_rtde",
        "@nlohmann_json//:json",
        "@drake//lcmtypes:schunk",
        "@gflags//:gflags",
        "@lcm//:lcm",
    ]
)
