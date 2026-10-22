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

# This makefile is the entrypoint of the build process. It chooses
# whether to execute a self-hosted build or an SDP cross-compiled build.
# It does this by detecting the environment variables that are set
# by the SDP development environment. If they are present, the SDP
# cross-compiled build is executred. Otherwise the self-hosted
# build is done.

ifneq ($(and $(QNX_TARGET),$(QNX_HOST)),)
$(info SDP cross-compiled build)
LIST=OS
include recurse.mk

else
$(info Self-hosted build)
include self-hosted.mk
endif
