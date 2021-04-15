#!/bin/bash

script="$(readlink -f $0)"
script_dir="$(dirname $script)"

. "$script_dir/config.sh.in"


if [ "$use_custom_application" = true ]; then
  if [ ! -d "$custom_application_source_dir" ]; then
    git clone "$custom_application_git_remote_address" -o upstream "$custom_application_name"
    pushd "$custom_application_source_dir"
    git checkout "$custom_application_git_version"
    popd
  fi
fi

