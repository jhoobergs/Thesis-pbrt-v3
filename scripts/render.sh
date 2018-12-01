#!/bin/bash

source $(dirname "$0")/render_simple.sh
diff $SCENE_PATH".png" $RESULTS_DIR/$FILE_NAME
xdg-open $RESULTS_DIR/$FILE_NAME
$BASE"/scripts/show.sh" $NAME
