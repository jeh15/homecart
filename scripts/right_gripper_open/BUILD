# BUILD
cc_binary(
    name = "right_gripper_open",
    srcs = ["right_gripper_open.cc",],
    linkstatic = True,
    linkopts = ["-lpthread"],
    deps = [
        "@nlohmann_json//:json",
        "@drake//lcmtypes:schunk",
        "@gflags//:gflags",
        "@lcm//:lcm",
    ]
)
