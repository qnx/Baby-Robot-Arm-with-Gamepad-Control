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

# This makefile is used by the self-hosted build (built on QNX)

RESOURCE_XML = physical-ai-ui.gresource.xml
RESOURCE_C = gen/ui_resources.c

SRCS = physical-ai-ui.c app_theme.c app_ui.c camera_runtime.c frame_pipeline.c \
       overlay_controls.c overlay_renderer.c pose_data.c user_settings.c $(RESOURCE_C)
CSC_SRCS += csc/csc_libyuv.c
CSC_LIBS += -lyuv
CJSON_LIBS += -lcjson
SYSTEM_FLAGS = -I/system/usr/include
SYSTEM_LIBS = -L/system/usr/lib

GTK_FLAGS = $(shell pkg-config --cflags gtk4)
GTK_LIBS = $(shell pkg-config --libs gtk4)

all: physical-ai-ui

$(RESOURCE_C): $(RESOURCE_XML) data/QNXLogo.png \
               data/QNXEverywhereLogo.png \
               data/robot_mode_screensaver.png \
               data/robot_mode_screensaver_cobra.png \
               data/robot_mode_screensaver_conductor.png \
               data/robot_mode_screensaver_figure_eight.png \
               data/robot_mode_screensaver_wave.png \
               data/robot_mode_joystick.png \
               data/robot_mode_hand_selected_left.png \
               data/robot_mode_hand_selected_right.png \
               data/robot_mode_joints_selected_left.png \
               data/robot_mode_joints_selected_right.png \
               data/robot_mode_joints_not_selected.png \
               data/robot_mode_hand_not_selected.png \
               data/qr-code-github.png \
               data/style.css
	mkdir -p $(dir $@)
	glib-compile-resources $(RESOURCE_XML) --sourcedir=$(CURDIR) --generate --target $@

physical-ai-ui: $(SRCS) $(CSC_SRCS)
	gcc -g $(GTK_FLAGS) $(CSC_FLAGS) $(SYSTEM_FLAGS) \
	    -o physical-ai-ui \
	    $(SRCS) $(CSC_SRCS) \
	    $(GTK_LIBS) $(CSC_LIBS) $(CJSON_LIBS) $(SYSTEM_LIBS) \
      -lcamapi -lz -lm

clean:
	$(RM) physical-ai-ui $(RESOURCE_C)
