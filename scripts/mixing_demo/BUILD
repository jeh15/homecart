# BUILD
cc_binary(
    name = "mixing_script",
    srcs = ["mixing_script.cc",],
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