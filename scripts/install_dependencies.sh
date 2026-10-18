#!/usr/bin/env bash
# Installs the dependencies of robotino-ros2 on Ubuntu (amd64):
#   1. Robotino API2 (rec-rpc, robotino-dev, robotino-api2) from the openrobotino.org package repository.
#      Upstream packages exist only for Ubuntu 20.04 (focal). Their libraries need only Qt5 Core/Network/Xml,
#      which newer Ubuntu releases ship under the *t64 package names, so the Depends field is rewritten
#      before installing.
#   2. ROS dependencies of the packages in this repository via rosdep.
#
# Usage: scripts/install_dependencies.sh [--api2-only] [--rosdistro <distro>]
#   --api2-only           install Robotino API2 only, skip rosdep
#   --rosdistro <distro>  ROS 2 distribution for rosdep (default: $ROS_DISTRO or the only one in /opt/ros)
set -euo pipefail

ROBOTINO_PKG_URL=https://packages2.openrobotino.org/pool/focal/main/r
# Package name, version and SHA-256 of the .deb files.
ROBOTINO_PKGS=(
    "rec-rpc 1.6.1 a4fdb236707cf2211fe8ec3c86c5caa2b7e30e6ebfdbe6a198f3b216860dafd5"
    "robotino-dev 1.0.3 704b07e475fec1de694fef0d6af668591831c48c04650694bdd84609a99c8fcc"
    "robotino-api2 1.1.14 a3371fa9caddc7a26226d75a36f52975c36885ca11d3815c3b5c02af008db896"
)

REPO_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
API2_ONLY=false
ROSDISTRO=${ROS_DISTRO:-}

usage() {
    sed -n 's/^# \{0,1\}//; 2,/^set /{/^set /d; p}' "${BASH_SOURCE[0]}"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --api2-only) API2_ONLY=true ;;
        --rosdistro) ROSDISTRO=${2:?--rosdistro needs a value}; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; usage >&2; exit 1 ;;
    esac
    shift
done

log() { echo -e "\033[1;32m==>\033[0m $*"; }
die() { echo -e "\033[1;31mError:\033[0m $*" >&2; exit 1; }

SUDO=()
if [[ $EUID -ne 0 ]]; then
    command -v sudo >/dev/null || die "run as root or install sudo"
    SUDO=(sudo)
fi
export DEBIAN_FRONTEND=noninteractive

install_api2() {
    [[ $(dpkg --print-architecture) == amd64 ]] || die "Robotino API2 is available only for amd64"

    log "Installing tools and Qt5 libraries"
    "${SUDO[@]}" apt-get update
    "${SUDO[@]}" apt-get install -y --no-install-recommends ca-certificates wget
    # Ubuntu 24.04+ renamed libqt5core5a etc. to libqt5core5t64 etc. (64-bit time_t transition).
    local rename_qt=false
    if apt-cache show libqt5core5t64 >/dev/null 2>&1; then
        rename_qt=true
        "${SUDO[@]}" apt-get install -y --no-install-recommends \
            libqt5core5t64 libqt5gui5t64 libqt5network5t64 libqt5xml5t64
    fi

    local tmp
    tmp=$(mktemp -d)
    trap "rm -rf '${tmp}'" EXIT
    mkdir -p "${tmp}/orig" "${tmp}/fixed"

    local entry name version sha deb debs=()
    for entry in "${ROBOTINO_PKGS[@]}"; do
        read -r name version sha <<<"${entry}"
        deb=${name}_${version}_amd64.deb
        log "Downloading ${deb}"
        wget -q -O "${tmp}/orig/${deb}" "${ROBOTINO_PKG_URL}/${name}/${deb}"
        echo "${sha}  ${tmp}/orig/${deb}" | sha256sum -c --quiet - || die "checksum mismatch for ${deb}"
        if ${rename_qt}; then
            dpkg-deb -R "${tmp}/orig/${deb}" "${tmp}/${name}"
            sed -i -E '/^Depends:/ { s/libqt5core5a\b/libqt5core5t64/; s/libqt5(gui5|network5|xml5)\b/libqt5\1t64/g }' \
                "${tmp}/${name}/DEBIAN/control"
            dpkg-deb -b "${tmp}/${name}" "${tmp}/fixed/${deb}" >/dev/null
            debs+=("${tmp}/fixed/${deb}")
        else
            debs+=("${tmp}/orig/${deb}")
        fi
    done

    log "Installing Robotino API2"
    "${SUDO[@]}" apt-get install -y --no-install-recommends "${debs[@]}"
    "${SUDO[@]}" ldconfig
    if ldd /opt/robotino/lib/librec_robotino_api2.so | grep "not found"; then
        die "librec_robotino_api2.so has unresolved libraries"
    fi
    log "Robotino API2 installed to /opt/robotino"
}

install_ros_deps() {
    if [[ -z ${ROSDISTRO} ]]; then
        local distros=(/opt/ros/*/)
        [[ ${#distros[@]} -eq 1 && -d ${distros[0]} ]] \
            || die "ROS 2 distribution not detected: install ROS 2 first, source it or pass --rosdistro"
        ROSDISTRO=$(basename "${distros[0]}")
    fi
    [[ -f /opt/ros/${ROSDISTRO}/setup.bash ]] || die "ROS 2 ${ROSDISTRO} is not installed in /opt/ros/${ROSDISTRO}"

    log "Installing rosdep and colcon"
    "${SUDO[@]}" apt-get install -y --no-install-recommends python3-rosdep python3-colcon-common-extensions
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
        "${SUDO[@]}" rosdep init
    fi
    # rosdep update must run as the user who later runs rosdep install.
    rosdep update --rosdistro "${ROSDISTRO}"

    log "Installing ROS dependencies of ${REPO_DIR} for ${ROSDISTRO}"
    rosdep install --from-paths "${REPO_DIR}" --ignore-src -y --rosdistro "${ROSDISTRO}"
}

install_api2
if ! ${API2_ONLY}; then
    install_ros_deps
fi
log "Done"
