import glob
import os
from invoke import task

THISDIR = os.path.dirname(os.path.realpath(__file__))

def get_submodules(c):
    """
    return repository submodule names
    """
    submodules = []
    with c.cd(THISDIR):
        result = c.run("git submodule status", hide=True)
        for line in result.stdout.splitlines():
            submodules.append(line.split()[1])
    return submodules

@task
def init(c):
    """
    Init submodules.
    """
    print("init submodules")
    with c.cd(THISDIR):
        c.run("git submodule init", hide=True)

@task(
    init,
    help={'public': "clone public submodules by default. Unset to clone all repositories."}
)
def clone(c, public=True):
    """
    Clone fog-sw repository submodules.
    """
    submodules = get_submodules(c)
    with c.cd(THISDIR):
        if public:
            submodules.remove('provisioning-agent')
            submodules.remove('ros2_ws/src/update-agent')
            submodules.remove('ros2_ws/src/mesh_com')
            c.run("git submodule update ros2_ws/src/mesh_com")
        for sub in submodules:
            c.run("git submodule update --init --recursive %s" %sub)

@task(
    help={'nocache': "do not use cache when building the image",
          'pull': "always attempt to pull a newer version of the image"}
)
def buildenv(c, nocache=False, pull=False):
    """
    Create fog-sw Docker build environment.
    """
    args = []
    args.append("--build-arg UID=$(id -u)")
    args.append("--build-arg GID=$(id -g)")
    args.append("-f Dockerfile")
    args.append("-t fogsw-builder:latest")
    if nocache:
        args.append("--no-cache")
    elif pull:
        args.append("--pull")
    with c.cd(THISDIR):
        c.run("docker build %s ." % " ".join(args))

@task(
    help={'reallyclean': "remove & reload all submodules"}
)
def clean(c, reallyclean=False):
    """
    Clean workspace.
    """
    with c.cd(THISDIR):
        if reallyclean:
            c.run("git submodule deinit -f --all")
            clone(c)
        else:
            c.run("git submodule foreach git clean -xdf")
            c.run("git submodule foreach git checkout .")
        c.run("git clean -xdf")

@task(buildenv)
def build_ros2(c):
    """
    Build fog-sw ROS2 modules
    """
    c.run("docker run --rm -v %s:/fog_sw fogsw-builder /fog_sw/packaging/package_ros.sh" % THISDIR)

@task(buildenv)
def build_system(c):
    """
    Build drivers and other system components
    """
    c.run("docker run --rm -v %s:/fog_sw fogsw-builder /fog_sw/packaging/package_sys.sh" % THISDIR)
    # fogsw-tools
    with c.cd("%s/fogsw_tools" % THISDIR):
        c.run("./build.sh %s/packaging/deb_files" % THISDIR)

@task(buildenv)
def build_kernel(c):
    """
    Build fog-sw kernel
    """
    c.run("docker run --rm -v %s:/fog_sw fogsw-builder /fog_sw/packaging/fogsw_kernel_config/package.sh" % THISDIR)

@task
def build_private(c):
    """
    Build fog-sw private packages
    """
    # provisioning-agent
    with c.cd("%s/provisioning-agent" % THISDIR):
        c.run("./build.sh %s/packaging/deb_files" % THISDIR)
    # update-agent
    with c.cd("%s/ros2_ws/src/update-agent" % THISDIR):
        c.run("./build.sh %s/packaging/deb_files" % THISDIR)

@task(
    help={'tag': "fog-drone docker image tag"}
)
def build_dronsole(c, tag="fog-drone"):
    """
    Build fog-drone docker image from locally build fog-sw packages
    """
    deb_files = glob.glob('%s/packaging/deb_files/*.deb' % THISDIR)
    if not deb_files:
        build_system(c)
        build_ros2(c)
    with c.cd(THISDIR):
        c.run("docker build -f Dockerfile.fog-drone -t %s ." %  tag)

@task(
    buildenv,
    help={'public': "Build public components by default. Unset to include private builds."}
)
def build(c, public=True):
    """
    Build all fog-sw components (public or private).
    """
    build_kernel(c)
    build_system(c)
    build_ros2(c)

    if not public:
        build_private(c)
