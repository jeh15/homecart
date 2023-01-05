# This package is public so that other packages can refer to
# individual files in these models from their bazel rules.
package(
    default_visibility = ["//visibility:public"],
)

# BUILD
cc_binary(
    name = "visualize",
    srcs = ["visualize.cc",],
    data = [
        "@drake//manipulation/models/tri_homecart:models",
        "@drake//manipulation/models/ur3e:models",
        "@drake//manipulation/models/wsg_50_description:models",
        ],
    linkstatic = True,
    linkopts = ["-lpthread"],
    deps = [
        "@ur_rtde",
        "@drake//lcmtypes:schunk",
        "@drake//common:find_resource",
        "@drake//multibody/parsing",
        "@drake//multibody/inverse_kinematics",
        "@drake//multibody/meshcat",
        "@drake//solvers:solve",
        "@drake//geometry",
        "@gflags//:gflags",
        "@lcm//:lcm",
    ],
)