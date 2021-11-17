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
    help={'public': "clone public submodules by default. Unset to clone all repositories.",
          'px4': "clone px4-firmware repository and submodules"}
)
def clone(c, public=True, px4=True):
    """
    Clone fog-sw repository submodules.
    """
    submodules = get_submodules(c)
    with c.cd(THISDIR):
        if not px4:
            submodules.remove('px4-firmware')
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
def buildenv_pixhawk(c, nocache=False, pull=False):
    """
    Create fog-sw/px4-firmware (pixhawk) Docker build environment.
    """
    buildenv(c, nocache, pull, pixhawk=True)

@task(
    help={'nocache': "do not use cache when building the image",
          'pull': "always attempt to pull a newer version of the image"}
)
def buildenv_saluki(c, nocache=False, pull=False):
    """
    Create fog-sw/px4-firmware (saluki) Docker build environment.
    """
    buildenv(c, nocache, pull, saluki=True)

@task(
    help={'nocache': "do not use cache when building the image",
          'pull': "always attempt to pull a newer version of the image"}
)
def buildenv_sitl(c, nocache=False, pull=False):
    """
    Create fog-sw/px4-firmware (sitl) Docker build environment.
    """
    buildenv(c, nocache, pull, sitl=True)


@task(
    help={'nocache': "do not use cache when building the image",
          'pull': "always attempt to pull a newer version of the image",
          'pixhawk': "px4-firmware/pixhawk build environment",
          'saluki': "px4-firmware/saluki build environment",
          'sitl': "px4-firmware/sitl build environment" }
)
def buildenv(c, nocache=False, pull=False, pixhawk=False, saluki=False, sitl=False):
    """
    Create fog-sw Docker build environment.
    """
    args = []
    args.append("--build-arg UID=$(id -u)")
    args.append("--build-arg GID=$(id -g)")
    args.append("-f Dockerfile")
    if nocache:
        args.append("--no-cache")
    elif pull:
        args.append("--pull")
    with c.cd(THISDIR):
        if pixhawk:
            args.append("-t pixhawk-builder:latest")
            args.append("-f Dockerfile.pixhawk")
        elif saluki:
            args.append("-t saluki-builder:latest")
            args.append("-f Dockerfile.saluki")
        elif sitl:
            args.append("-t sitl-builder:latest")
            args.append("-f Dockerfile.sitl")
        else:
            args.append("-t fogsw-builder:latest")
        c.run("docker build %s ." % " ".join(args))

@task(
    help={'reallyclean': "remove & reload all submodules. Clean docker build images"}
)
def clean(c, reallyclean=False):
    """
    Clean workspace.
    """
    with c.cd(THISDIR):
        if reallyclean:
            c.run("git clean -xdf")
            c.run("git submodule deinit -f --all")
            c.run("docker rmi fogsw-builder")
            c.run("docker rmi pixhawk-builder")
            c.run("docker rmi saluki-builder")
            c.run("docker rmi sitl-builder")
            c.run("docker rmi fog-drone")
            clone(c)
        else:
            c.run("git submodule foreach git clean -xdf")
            c.run("git submodule foreach git checkout .")
            c.run("git clean -fd")

@task(buildenv_pixhawk)
def build_pixhawk(c):
    """
    Build px4-firmware pixhawk
    """
    if not os.path.exists('%s/tools/Fast-DDS-Gen/share/fastddsgen/java/fastddsgen.jar' % THISDIR):
        build_tools(c)
    c.run("docker run --rm -v %s:/fog_sw pixhawk-builder packaging/pixhawk/package.sh" % THISDIR)

@task(buildenv_saluki)
def build_saluki(c):
    """
    Build px4-firmware saluki
    """
    if not os.path.exists('%s/tools/Fast-DDS-Gen/share/fastddsgen/java/fastddsgen.jar' % THISDIR):
        build_tools(c)
    c.run("docker run --rm -v %s:/fog_sw saluki-builder packaging/saluki/package.sh" % THISDIR)

@task(buildenv_sitl)
def build_sitl(c):
    """
    Build px4-firmware sitl
    """
    if not os.path.exists('%s/tools/Fast-DDS-Gen/share/fastddsgen/java/fastddsgen.jar' % THISDIR):
        build_tools(c)
    c.run("docker run --rm -v %s:/fog_sw sitl-builder packaging/sitl/package.sh" % THISDIR)

@task(buildenv)
def build_px4fwupdater(c):
    """
    Build px4-fwupdater
    """
    if not glob.glob('%s/packaging/px4_files/px4_fmu-v5*.px4' % THISDIR):
        build_pixhawk(c)
    if not glob.glob('%s/packaging/px4_files/ssrc_saluki-v1_default*.px4' % THISDIR):
        build_saluki(c)
    c.run("docker run --rm -v %s:/fog_sw fogsw-builder /fog_sw/packaging/px4fwupdater/package.sh" % THISDIR)

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

@task(buildenv)
def build_kernel(c):
    """
    Build fog-sw kernel
    """
    c.run("docker run --rm -v %s:/fog_sw fogsw-builder /fog_sw/packaging/fogsw_kernel_config/package.sh" % THISDIR)

@task(buildenv)
def build_tools(c):
    """
    Build fog-sw tools
    """
    c.run("docker run --rm -v %s:/fog_sw fogsw-builder /fog_sw/packaging/package_tools.sh" % THISDIR)

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
    if not glob.glob('%s/packaging/deb_files/*.deb' % THISDIR):
        build_system(c)
        build_ros2(c)
    if not glob.glob('%s/packaging/px4_files/px4_*.tar.gz' % THISDIR):
        build_sitl(c)
    with c.cd(THISDIR):
        c.run("docker build -f Dockerfile.fog-drone -t %s ." %  tag)

@task(
    buildenv,
    buildenv_pixhawk,
    buildenv_saluki,
    buildenv_sitl,
    help={'public': "Build public components by default. Unset to include private builds."}
)
def build(c, public=True):
    """
    Build all fog-sw components (public or private).
    """
    build_tools(c)
    build_pixhawk(c)
    build_saluki(c)
    build_px4fwupdater(c)
    build_kernel(c)
    build_system(c)
    build_ros2(c)

    if not public:
        build_private(c)
