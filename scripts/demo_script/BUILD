# BUILD
cc_binary(
    name = "demo_script",
    srcs = ["demo_script.cc",],
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