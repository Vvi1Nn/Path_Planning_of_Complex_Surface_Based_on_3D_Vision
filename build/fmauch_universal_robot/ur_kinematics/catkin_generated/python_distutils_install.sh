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
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/zwh/zwh_main/src/fmauch_universal_robot/ur_kinematics"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/zwh/zwh_main/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/zwh/zwh_main/install/lib/python3/dist-packages:/home/zwh/zwh_main/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/zwh/zwh_main/build" \
    "/usr/bin/python3" \
    "/home/zwh/zwh_main/src/fmauch_universal_robot/ur_kinematics/setup.py" \
     \
    build --build-base "/home/zwh/zwh_main/build/fmauch_universal_robot/ur_kinematics" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/zwh/zwh_main/install" --install-scripts="/home/zwh/zwh_main/install/bin"
