#
# Copyright (c) 2026, BlackBerry Limited. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# This makefile is used by the SDP cross-compiled build. It is included
# automatically by the recursive build system from the various leaf directories
# (eg: nto/aarch64/o.le)

# The basic QNX makefile definition
ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

# A short description of the binary
define PINFO
PINFO DESCRIPTION=Physical AI UI in GTK
endef

# The location to install the built binary on a target
INSTALLDIR=usr/bin
NAME=physical-ai-ui

RESOURCE_XML=$(PROJECT_ROOT)/physical-ai-ui.gresource.xml
RESOURCE_C=$(PROJECT_ROOT)/gen/ui_resources.c

# Adding files to SRCS means I take ownership of creating the full list.
# Recursive make will no longer automatically add the SRCVPATH dirs.
EXTRA_SRCVPATH += $(PROJECT_ROOT) $(PROJECT_ROOT)/csc
LATE_SRCVPATH += $(PROJECT_ROOT)/gen
SRCS = physical-ai-ui.c app_theme.c app_ui.c camera_runtime.c frame_pipeline.c \
       overlay_controls.c overlay_renderer.c pose_data.c user_settings.c csc_libyuv.c
EXTRA_OBJS = ui_resources.o
EXTRA_INCVPATH += $(INSTALL_ROOT_nto)/usr/include
EXTRA_LIBVPATH += $(INSTALL_ROOT_nto)/$(CPUVARDIR)/usr/lib
LIBS += yuv cjson

# Add include path and libraries for GTK
EXTRA_INCVPATH += $(INSTALL_ROOT_nto)/$(CPUVARDIR)/usr/local/include/gtk-4.0 \
                  $(INSTALL_ROOT_nto)/$(CPUVARDIR)/usr/local/include/pango-1.0 \
                  $(INSTALL_ROOT_nto)/$(CPUVARDIR)/usr/local/include/fribidi \
                  $(INSTALL_ROOT_nto)/$(CPUVARDIR)/usr/local/include/harfbuzz \
                  $(INSTALL_ROOT_nto)/$(CPUVARDIR)/usr/local/include/gdk-pixbuf-2.0 \
                  $(INSTALL_ROOT_nto)/$(CPUVARDIR)/usr/local/include/webp \
                  $(INSTALL_ROOT_nto)/$(CPUVARDIR)/usr/local/include/cairo \
                  $(INSTALL_ROOT_nto)/$(CPUVARDIR)/usr/local/include/freetype2 \
                  $(INSTALL_ROOT_nto)/$(CPUVARDIR)/usr/local/include/libpng16 \
                  $(INSTALL_ROOT_nto)/$(CPUVARDIR)/usr/local/include/pixman-1 \
                  $(INSTALL_ROOT_nto)/$(CPUVARDIR)/usr/local/include/graphene-1.0 \
                  $(INSTALL_ROOT_nto)/$(CPUVARDIR)/usr/local/lib/graphene-1.0/include \
                  $(INSTALL_ROOT_nto)/$(CPUVARDIR)/usr/local/include/glib-2.0 \
                  $(INSTALL_ROOT_nto)/$(CPUVARDIR)/usr/local/lib/glib-2.0/include
CPPFLAGS += -DWITH_GZFILEOP
LIBS += gtk-4 pangocairo-1.0 pango-1.0 harfbuzz gdk_pixbuf-2.0 cairo-gobject cairo graphene-1.0 gio-2.0 gobject-2.0 glib-2.0
EXTRA_LIBVPATH += $(INSTALL_ROOT_nto)/$(CPUVARDIR)/usr/local/lib

# Add camera API library
LIBS += camapi m

# Further QNX makefile definitions
include $(MKFILES_ROOT)/qmacros.mk
include $(MKFILES_ROOT)/qtargets.mk

$(RESOURCE_C): $(RESOURCE_XML) $(PROJECT_ROOT)/data/QNXLogo.png \
               $(PROJECT_ROOT)/data/QNXEverywhereLogo.png \
               $(PROJECT_ROOT)/data/robot_mode_screensaver.png \
               $(PROJECT_ROOT)/data/robot_mode_screensaver_cobra.png \
               $(PROJECT_ROOT)/data/robot_mode_screensaver_conductor.png \
               $(PROJECT_ROOT)/data/robot_mode_screensaver_figure_eight.png \
               $(PROJECT_ROOT)/data/robot_mode_screensaver_wave.png \
               $(PROJECT_ROOT)/data/robot_mode_joystick.png \
               $(PROJECT_ROOT)/data/robot_mode_hand_selected_left.png \
               $(PROJECT_ROOT)/data/robot_mode_hand_selected_right.png \
               $(PROJECT_ROOT)/data/robot_mode_joints_selected_left.png \
               $(PROJECT_ROOT)/data/robot_mode_joints_selected_right.png \
               $(PROJECT_ROOT)/data/robot_mode_joints_not_selected.png \
               $(PROJECT_ROOT)/data/robot_mode_hand_not_selected.png \
               $(PROJECT_ROOT)/data/qr-code-github.png \
               $(PROJECT_ROOT)/data/style.css
	mkdir -p $(dir $@)
	glib-compile-resources $(RESOURCE_XML) --sourcedir=$(PROJECT_ROOT) --generate --target $@
