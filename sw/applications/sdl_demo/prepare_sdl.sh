#!/bin/bash

# Tool to prepare SDL2 and asterics
# Path points to project top level
cd ../../

PARANUT=$(pwd)
ASTERICS_REMOTE_URL=https://ti-build.informatik.hs-augsburg.de:8443/asterics_developers/asterics.git
SDL_VERSION=2.0.22
SDL_DOWNLOAD_URL=https://libsdl.org/release/SDL2-$SDL_VERSION.tar.gz
EXTERNAL=$PARANUT/external
SDL=$EXTERNAL/SDL2
ASTERICS=$EXTERNAL/asterics
PATCHES=$EXTERNAL/patches

PREPARE_SDL=1

if [ -d "$SDL" ]; then
    read -n1 -p "Old SDL2 folder found, will delete. Continue? [y\N]: " ret_val
    echo ""
    if [ "$ret_val" != "y" ]; then
        echo "Aborting..."
        PREPARE_SDL=0
    fi
fi

if [ $PREPARE_SDL == 1 ]; then
    echo "Installing SDL2"
    rm -rf "$SDL"
    if [ ! -f "downloads/SDL2-$SDL_VERSION.tar.gz" ]; then
        mkdir downloads
        echo "Downloading SDL2"
        wget $SDL_DOWNLOAD_URL -O downloads/SDL2-$SDL_VERSION.tar.gz
    else
        echo "Using SDL2-$SDL_VERSION.tar.gz in downloads folder."
    fi

    echo "Extracting SDL2"
    tar xf downloads/SDL2-$SDL_VERSION.tar.gz
    mv SDL2-$SDL_VERSION $SDL
    (cd $EXTERNAL && patch -p0 < $PATCHES/SDL2.patch)
fi

## Create and patch asterics repo

if [ -d "$ASTERICS" ]; then
    echo "ASTERICS repository already present. Assuming it's patched already."
else
    echo "About to clone ASTERICS to $ASTERICS"
    git clone $ASTERICS_REMOTE_URL $ASTERICS
    if (cd $ASTERICS && git checkout 51cced9c2f158ba19cf6b8ca2e62633a6609a7bd); then
        echo "Patching VEARS code for ParaNut..."
        if (cd $ASTERICS/ipcores/VEARS/drivers/vears_v1_0 && patch -p0 < $PATCHES/vears.patch); then
            echo "Successfully patched"
        else
            echo "ERROR: PATCH FAILED!"
            exit 1;
        fi
    fi
fi
