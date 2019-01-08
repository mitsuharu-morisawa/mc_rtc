#!/bin/bash -ex

shopt -s expand_aliases

##########################
#  --  Configuration --  #
##########################

readonly mc_rtc_dir=`cd $(dirname $0)/..; pwd`

readonly SOURCE_DIR=`cd $mc_rtc_dir/../; pwd`

#default settings
INSTALL_PREFIX="/usr/local"
WITH_ROS_SUPPORT="true"
WITH_PYTHON_SUPPORT="true"
PYTHON_USER_INSTALL="false"
WITH_HRP2="true"
WITH_HRP4="true"
VREP_PATH=
BUILD_TYPE="RelWithDebInfo"
INSTALL_APT_DEPENDENCIES="true"
if command -v nproc > /dev/null
then
   BUILD_CORE=`nproc`
else
   BUILD_CORE=`sysctl -n hw.ncpu`
fi

if [ `lsb_release -sc` = "trusty" ]
then
  ROS_DISTRO=indigo
else
  ROS_DISTRO=kinetic
fi

readonly HELP_STRING="$(basename $0) [OPTIONS] ...
    --help                     (-h)               : print this help
    --install-prefix           (-i) PATH          : the directory used to install everything         (default $INSTALL_PREFIX)
    --build-type                    Type          : the build type to use                            (default $BUILD_TYPE)
    --build-core               (-j) N             : number of cores used for building                (default $BUILD_CORE)
    --with-hrp2                                   : enable HRP2 (requires mc-hrp2 group access)      (default $WITH_HRP2)
    --with-hrp4                                   : enable HRP4 (requires mc-hrp4 group access)      (default $WITH_HRP4)
    --with-python-support           {true, false} : whether to build with Python support             (default $WITH_PYTHON_SUPPORT)
    --python-user-install           {true, false} : whether to install Python bindings with user     (default $PYTHON_USER_INSTALL)
    --with-ros-support              {true, false} : whether to build with ros support                (default $WITH_ROS_SUPPORT)
    --ros-distro                    NAME          : the ros distro to use                            (default $ROS_DISTRO)
    --install-apt-dependencies      {true, false} : whether to install packages                      (default $INSTALL_APT_DEPENDENCIES)
    --vrep-path                     PATH          : where to find vrep (will be downloaded if empty) (default $VREP_PATH)
"

#helper for parsing
check_true_false()
{
    if [ "true" != "$2" ] && [ "false" != "$2" ]
    then
        echo "passed parameter '$2' as flag for '$1'. the parameter has to be 'true' or 'false'"
        exit 1
    fi
}
#parse arguments
i=1
while [[ $# -ge $i ]]
do
    key="${!i}"
    case $key in
        -h|--help)
        echo "$HELP_STRING"
        exit
        ;;

        -i|--install-prefix)
        i=$(($i+1))
        INSTALL_PREFIX="${!i}"
        ;;

        --with-ros-support)
        i=$(($i+1))
        WITH_ROS_SUPPORT="${!i}"
        check_true_false --with-ros-support "$WITH_ROS_SUPPORT"
        ;;

        --with-python-support)
        i=$(($i+1))
        WITH_PYTHON_SUPPORT="${!i}"
        check_true_false --with-python-support "$WITH_PYTHON_SUPPORT"
        ;;

        --python-user-install)
        i=$(($i+1))
        PYTHON_USER_INSTALL="${!i}"
        check_true_false --python-user-install "$PYTHON_USER_INSTALL"
        ;;

      --with-hrp2)
        i=$(($i+1))
        WITH_HRP2="${!i}"
        check_true_false --with-hrp2 "$WITH_HRP2"
        ;;

      --with-hrp4)
        i=$(($i+1))
        WITH_HRP4="${!i}"
        check_true_false --with-hrp4 "$WITH_HRP4"
        ;;

        --build-type)
        i=$(($i+1))
        BUILD_TYPE="${!i}"
        ;;

        --install-apt-dependencies)
        i=$(($i+1))
        INSTALL_APT_DEPENDENCIES="${!i}"
        check_true_false --install-apt-dependencies "$INSTALL_APT_DEPENDENCIES"
        ;;

        -j|--build-core)
        i=$(($i+1))
        BUILD_CORE="${!i}"
        ;;

        --ros-distro)
        i=$(($i+1))
        ROS_DISTRO="${!i}"
        ;;

        --vrep-path)
        i=$(($i+1))
        VREP_PATH="${!i}"
        ;;

        *)
        echo "unknown parameter $i ($key)"
        exit 1
        ;;
    esac

    i=$(($i+1))
done
if $WITH_PYTHON_SUPPORT
then
  WITH_PYTHON_SUPPORT=ON
else
  WITH_PYTHON_SUPPORT=OFF
fi
if $PYTHON_USER_INSTALL
then
  PYTHON_USER_INSTALL=ON
else
  PYTHON_USER_INSTALL=OFF
fi
#make settings readonly
readonly INSTALL_PREFIX
readonly WITH_ROS_SUPPORT
readonly WITH_PYTHON_SUPPORT
readonly PYTHON_USER_INSTALL
readonly BUILD_TYPE
readonly INSTALL_APT_DEPENDENCIES
readonly BUILD_CORE

readonly ROS_APT_DEPENDENCIES="ros-${ROS_DISTRO}-common-msgs ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-rviz"
ROS_GIT_DEPENDENCIES="git@gite.lirmm.fr:multi-contact/mc_rtc_ros_data#master"
if $WITH_HRP2
then
  ROS_GIT_DEPENDENCIES="$ROS_GIT_DEPENDENCIES git@gite.lirmm.fr:mc-hrp2/hrp2_drc#master"
fi
if $WITH_HRP4
then
  ROS_GIT_DEPENDENCIES="$ROS_GIT_DEPENDENCIES git@gite.lirmm.fr:mc-hrp4/hrp4#master"
fi
alias git_clone="git clone --quiet --recursive"
alias git_update="git pull && git submodule update"

SUDO_CMD='sudo -E'
PIP_USER=
if [ -w $INSTALL_PREFIX ]
then
  SUDO_CMD=
  PIP_USER='--user'
fi

readonly gitlab_ci_yml=$mc_rtc_dir/.gitlab-ci.yml

export PATH=$INSTALL_PREFIX/bin:$PATH
export LD_LIBRARY_PATH=$INSTALL_PREFIX/lib:$LD_LIBRARY_PATH
export DYLD_LIBRARY_PATH=$INSTALL_PREFIX/lib:$DYLD_LIBRARY_PATH
export PKG_CONFIG_PATH=$INSTALL_PREFIX/lib/pkgconfig:$PKG_CONFIG_PATH
export PYTHONPATH=$INSTALL_PREFIX/lib/python2.7/site-packages:$PYTHONPATH

yaml_to_env()
{
  local var=$1
  local f=$2
  tmp=`grep "$var:" $f|sed -e"s/.*${var}: \"\(.*\)\"/\1/"`
  export $var="$tmp"
}

##############################
#  --  APT/Brew dependencies  --  #
##############################
KERN=$(uname -s)
if [ $KERN = Darwin ]
then
  export OS=Darwin
  # Install brew on the system
  if $INSTALL_APT_DEPENDENCIES
  then
    if ! command -v brew
    then
      /usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
    fi
    brew update
    brew install coreutils pkg-config gnu-sed wget gcc python cmake doxygen libtool tinyxml2 geos boost eigen || true
    # eigen3.pc is broken in brew release
    gsed -i -e's@Cflags: -Iinclude/eigen3@Cflags: -I/usr/local/include/eigen3@' /usr/local/lib/pkgconfig/eigen3.pc
  else
    echo "SKIPPING INSTALLATION OF BREW DEPENDENCIES"
  fi
else
  export OS=$(lsb_release -si)
  if [ $OS = Ubuntu ]
  then
    yaml_to_env "APT_DEPENDENCIES" $gitlab_ci_yml
    APT_DEPENDENCIES=`echo $APT_DEPENDENCIES|sed -e's/libspacevecalg-dev//'|sed -e's/librbdyn-dev//'|sed -e's/libeigen-qld-dev//'|sed -e's/libsch-core-dev//'|sed -e's/libtasks-qld-dev//'|sed -e's/libmc-rbdyn-urdf-dev//'|sed -e's/python-tasks//'|sed -e's/python-mc-rbdyn-urdf//'`
    APT_DEPENDENCIES="cmake build-essential gfortran doxygen libeigen3-dev python-pip $APT_DEPENDENCIES"
    if $INSTALL_APT_DEPENDENCIES
    then
        sudo apt-get update
        sudo apt-get install -qq ${APT_DEPENDENCIES}
    else
        echo "SKIPPING INSTALLATION OF APT_DEPENDENCIES ($APT_DEPENDENCIES)"
    fi
  else
    echo "This script does not support your OS: ${OS}, please contact the maintainer"
    exit 1
  fi
fi

git_dependency_parsing()
{
  _input=$1
  git_dep=${_input%%#*}
  git_dep_branch=${_input##*#}
  if [ "$git_dep_branch" = "$git_dep" ]; then
    if [ -e "$2" ]; then
      git_dep_branch=$2
    else
      git_dep_branch="master"
    fi
  fi
  git_dep_uri_base=${git_dep%%:*}
  if [ "$git_dep_uri_base" = "$git_dep" ]; then
    git_dep_uri="git://github.com/$git_dep"
  else
    git_dep_uri=$git_dep
    git_dep=${git_dep##*:}
  fi
  git_dep=`basename $git_dep`
}

build_git_dependency()
{
  git_dependency_parsing $1
  echo "--> Compiling $git_dep (branch $git_dep_branch)"
  cd "$SOURCE_DIR"
  mkdir -p "$git_dep"
  if [ ! -d "$git_dep/.git" ]
  then
    git_clone -b $git_dep_branch "$git_dep_uri" "$git_dep"
  else
    pushd .
    cd "$git_dep"
    git_update
    popd
  fi
  mkdir -p $git_dep/build
  cd "$git_dep/build"
  cmake .. -DCMAKE_INSTALL_PREFIX:STRING="$INSTALL_PREFIX" \
           -DPYTHON_BINDING:BOOL=${WITH_PYTHON_SUPPORT} \
           -DPYTHON_BINDING_USER_INSTALL:BOOL=${PYTHON_USER_INSTALL} \
           -DCMAKE_BUILD_TYPE:STRING="$BUILD_TYPE" \
           ${CMAKE_ADDITIONAL_OPTIONS}
  make -j${BUILD_CORE}
  ${SUDO_CMD} make install
}
###############################
##  --  GIT dependencies  --  #
###############################
yaml_to_env "GIT_DEPENDENCIES" $gitlab_ci_yml
# Add some source dependencies
if [ "x$WITH_PYTHON_SUPPORT" == xON ]
then
  GIT_DEPENDENCIES="jrl-umi3218/Eigen3ToPython jrl-umi3218/SpaceVecAlg jrl-umi3218/RBDyn jrl-umi3218/eigen-qld jrl-umi3218/sch-core jrl-umi3218/sch-core-python jrl-umi3218/mc_rbdyn_urdf ${GIT_DEPENDENCIES}"
else
  GIT_DEPENDENCIES="jrl-umi3218/SpaceVecAlg jrl-umi3218/RBDyn jrl-umi3218/eigen-qld jrl-umi3218/sch-core jrl-umi3218/mc_rbdyn_urdf ${GIT_DEPENDENCIES}"
fi
for package in ${GIT_DEPENDENCIES}; do
  build_git_dependency "$package"
done

################################
#  -- Handle ROS packages  --  #
################################
if $WITH_ROS_SUPPORT
then
  if [ ! -e /opt/ros/${ROS_DISTRO}/setup.bash ]
  then
    if [ $OS = Ubuntu ]
    then
      sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -c -s` main" > /etc/apt/sources.list.d/ros-latest.list'
      wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
      sudo apt-get update
      sudo apt-get install -qq ros-${ROS_DISTRO}-ros-base ros-${ROS_DISTRO}-rosdoc-lite python-catkin-lint ${ROS_APT_DEPENDENCIES}
    else
      echo "Please install ROS and the required dependencies (${ROS_APT_DEPENDENCIES}) before continuing your installation or disable ROS support"
      exit 1
    fi
  fi
  . /opt/ros/${ROS_DISTRO}/setup.bash
  CATKIN_SRC_DIR=$SOURCE_DIR/catkin_ws/src
  mkdir -p $CATKIN_SRC_DIR
  cd $CATKIN_SRC_DIR
  catkin_init_workspace || true
  for package in ${ROS_GIT_DEPENDENCIES}; do
    git_dependency_parsing $package
    cd $CATKIN_SRC_DIR
    if [ ! -d "$git_dep/.git" ]
    then
      git_clone -b $git_dep_branch "$git_dep_uri" "$git_dep"
    else
      cd "$git_dep"
      git_update
    fi
  done
  cd $SOURCE_DIR/catkin_ws
  catkin_make
  . $SOURCE_DIR/catkin_ws/devel/setup.bash
else
  ROS_GIT_DEPENDENCIES=`echo $ROS_GIT_DEPENDENCIES|sed -e's/hrp4#master/hrp4#noxacro/'`
  for package in ${ROS_GIT_DEPENDENCIES}; do
    git_dependency_parsing $package
    cd $SOURCE_DIR
    if [ ! -d "$git_dep/.git" ]
    then
      git_clone -b $git_dep_branch "$git_dep_uri" "$git_dep"
    else
      cd "$git_dep"
      git_update
    fi
  done
fi

##########################
#  --  Build mc_rtc  --  #
##########################
cd $mc_rtc_dir
git submodule update --init
mkdir -p build
cd build
if $WITH_ROS_SUPPORT
then
  cmake ../ -DCMAKE_BUILD_TYPE:STRING="$BUILD_TYPE" \
            -DCMAKE_INSTALL_PREFIX:STRING="$INSTALL_PREFIX" \
            -DPYTHON_BINDING:BOOL=${WITH_PYTHON_SUPPORT} \
            -DPYTHON_BINDING_USER_INSTALL:BOOL=${PYTHON_USER_INSTALL} \
            ${CMAKE_ADDITIONAL_OPTIONS}
else
  CMAKE_ROBOT_OPTIONS=""
  if $WITH_HRP2
  then
    CMAKE_ROBOT_OPTIONS="-DHRP2_DRC_DESCRIPTION_PATH:STRING='${SOURCE_DIR}/hrp2_drc/hrp2_drc_description'"
  fi
  if $WITH_HRP4
  then
    CMAKE_ROBOT_OPTIONS="$CMAKE_ROBOT_OPTIONS -DHRP4_DESCRIPTION_PATH:STRING='${SOURCE_DIR}/hrp4/hrp4_description'"
  fi
  cmake ../ -DCMAKE_BUILD_TYPE:STRING="'$BUILD_TYPE'" \
            -DCMAKE_INSTALL_PREFIX:STRING="'$INSTALL_PREFIX'" \
            -DPYTHON_BINDING:BOOL=${WITH_PYTHON_SUPPORT} \
            -DPYTHON_BINDING_USER_INSTALL:BOOL=${PYTHON_USER_INSTALL} \
            -DMC_ENV_DESCRIPTION_PATH:STRING="'$SOURCE_DIR/mc_rtc_ros_data/mc_env_description'" \
            ${CMAKE_ROBOT_OPTIONS} \
            ${CMAKE_ADDITIONAL_OPTIONS} \
            -DDISABLE_ROS=ON
fi
make -j$BUILD_CORE
${SUDO_CMD} make install

##############################
#  --  Build mc_rtc_ros  --  #
##############################
if $WITH_ROS_SUPPORT
then
  if $INSTALL_APT_DEPENDENCIES
  then
    if [ `apt-cache search libglfw3-dev|wc -l` -eq 0 ]
    then
      sudo add-apt-repository ppa:keithw/glfw3
      sudo apt-get update
    fi
    sudo apt-get install -qq libglfw3-dev
  fi
  CATKIN_DIR=$SOURCE_DIR/catkin_ws
  cd $CATKIN_DIR/src
  if [ ! -d mc_rtc_ros/.git ]
  then
    git_clone -b topic/SN git@gite.lirmm.fr:multi-contact/mc_rtc_ros
  else
    cd mc_rtc_ros
    git_update
  fi
  cd $CATKIN_DIR
  catkin_make
  . $CATKIN_DIR/devel/setup.bash
fi

####################################################
#  -- Setup VREP, vrep-api-wrapper and mc_vrep --  #
####################################################
if [ -z "${VREP_PATH}" ]
then
  VREP_MAJOR="V-REP_PRO_EDU_V3_4_0"
  cd $SOURCE_DIR
  if [ $OS = Darwin ]
  then
    VREP_MACOS="${VREP_MAJOR}_Mac"
    if [ ! -d $VREP_MACOS ]
    then
      wget http://coppeliarobotics.com/files/${VREP_MACOS}.zip
      unzip ${VREP_MACOS}.zip
    fi
    VREP_PATH=$SOURCE_DIR/$VREP_MACOS
  else
    if [ "`uname -i`" != "x86_64" ]
    then
      VREP_MAJOR="V-REP_PRO_EDU_V3_3_2"
      echo "[WARNING] VREP support for 32 bits stopped after 3.3.2, it might not work properly with the models or softwares we provide"
    fi
    VREP_LINUX="${VREP_MAJOR}_Linux"
    if [ ! -d ${VREP_LINUX} ]
    then
      wget http://coppeliarobotics.com/files/${VREP_LINUX}.tar.gz
      tar xzf ${VREP_LINUX}.tar.gz
    fi
    VREP_PATH=$SOURCE_DIR/$VREP_LINUX
  fi
fi
[ ! -e "$SOURCE_DIR/vrep" ] && ln -s "$VREP_PATH" "$SOURCE_DIR/vrep"

cd $SOURCE_DIR
if [ ! -d vrep-api-wrapper/.git ]
then
  git_clone git@gite.lirmm.fr:vrep-utils/vrep-api-wrapper
  cd vrep-api-wrapper
else
  cd vrep-api-wrapper
  git_update
fi
mkdir -p build && cd build
cmake ../ -DCMAKE_BUILD_TYPE:STRING="$BUILD_TYPE" \
          -DCMAKE_INSTALL_PREFIX:STRING="$INSTALL_PREFIX" \
          -DVREP_PATH:STRING="$VREP_PATH" \
          ${CMAKE_ADDITIONAL_OPTIONS}
make
${SUDO_CMD} make install

cd $SOURCE_DIR
if [ ! -d mc_vrep/.git ]
then
  git_clone git@gite.lirmm.fr:multi-contact/mc_vrep
  cd mc_vrep
else
  cd mc_vrep
  git_update
fi
mkdir -p build && cd build
cmake ../ -DCMAKE_BUILD_TYPE:STRING="$BUILD_TYPE" \
          -DCMAKE_INSTALL_PREFIX:STRING="$INSTALL_PREFIX" \
          ${CMAKE_ADDITIONAL_OPTIONS}
make
${SUDO_CMD} make install

cd $SOURCE_DIR
if $WITH_HRP4
then
  if [ ! -d vrep-hrp4/.git ]
  then
    git_clone git@gite.lirmm.fr:mc-hrp4/vrep_hrp.git vrep-hrp4
  else
    cd vrep-hrp4
    git_update
  fi
fi

cd $SOURCE_DIR
if $WITH_HRP2
then
  if [ ! -d vrep-hrp2/.git ]
  then
    git_clone git@gite.lirmm.fr:mc-hrp2/vrep-hrp2.git
  else
    cd vrep-hrp2
    git_update
  fi
fi

echo "Installation finished, please add the following lines to your .bashrc/.zshrc"
if [ ${OS} = Darwin ]
then
  echo """
  export PATH=$INSTALL_PREFIX/bin:\$PATH
  export DYLD_LIBRARY_PATH=$INSTALL_PREFIX/lib:\$DYLD_LIBRARY_PATH
  export PKG_CONFIG_PATH=$INSTALL_PREFIX/lib/pkgconfig:\$PKG_CONFIG_PATH
  export PYTHONPATH=$INSTALL_PREFIX/lib/python2.7/site-packages:\$PYTHONPATH
  """
else
  echo """
  export PATH=$INSTALL_PREFIX/bin:\$PATH
  export LD_LIBRARY_PATH=$INSTALL_PREFIX/lib:\$LD_LIBRARY_PATH
  export PKG_CONFIG_PATH=$INSTALL_PREFIX/lib/pkgconfig:\$PKG_CONFIG_PATH
  export PYTHONPATH=$INSTALL_PREFIX/lib/python2.7/site-packages:\$PYTHONPATH
  """
  if $WITH_ROS_SUPPORT
  then
    echo "source $SOURCE_DIR/catkin_ws/devel/setup.bash"
  fi
fi
