#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/egronda/sunfest_2019/src/catkin_ws/src/rpg_dvs_ros/dvs_calibration_gui"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/egronda/sunfest_2019/src/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/egronda/sunfest_2019/src/catkin_ws/install/lib/python2.7/dist-packages:/home/egronda/sunfest_2019/src/catkin_ws/build/dvs_calibration_gui/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/egronda/sunfest_2019/src/catkin_ws/build/dvs_calibration_gui" \
    "/usr/bin/python2" \
    "/home/egronda/sunfest_2019/src/catkin_ws/src/rpg_dvs_ros/dvs_calibration_gui/setup.py" \
    build --build-base "/home/egronda/sunfest_2019/src/catkin_ws/build/dvs_calibration_gui" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/egronda/sunfest_2019/src/catkin_ws/install" --install-scripts="/home/egronda/sunfest_2019/src/catkin_ws/install/bin"
