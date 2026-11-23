#!/bin/bash

LOG_FILE="cache/installer-log.log"
TEMP_FILE="$(mktemp)"

# Read terminal size for whiptail
read WHIPTAIL_ROWS WHIPTAIL_COLS < <(stty size)

if whiptail --title "Clean Log File" \
  --yesno "This will remove ALL escape sequences and blank lines from:\n\n$LOG_FILE\n\nAre you sure you want to proceed?" \
  $WHIPTAIL_ROWS $WHIPTAIL_COLS --yes-button "Yes" --no-button "No"; then

  # Nuclear option regex - removes nearly all ANSI sequences
  sed -r '
    s/\x1B\[([0-9]{1,3}(;[0-9]{1,3})*)?[mGK]//g;     # Standard CSI codes
    s/\x1B\[?[0-9;]*[hl]//g;                         # Mode changes
    s/\x1B\]8;;[^\x07]*\x07//g;                      # OSC hyperlinks (xterm)
    s/\x1B\][0-9][^\x07]*\x07//g;                    # OSC sequences (terminated by BEL)
    s/\x1B\][0-9;]*\x1B\\//g;                        # OSC sequences (terminated by ST)
    s/\x1B[()][^A-Za-z]*[A-Za-z]//g;                 # Character set changes
    s/\x1B%[^A-Za-z]*[A-Za-z]//g;                    # Encoding system changes
    s/\x1B[^[()%]*[A-Za-z]//g;                       # Catch-all for other escapes
    s/\x1B//g;                                        # Any remaining ESC characters
  ' "$LOG_FILE" | sed '/^[[:space:]]*$/d' > "$TEMP_FILE"

  # Force overwrite without confirmation
  \mv -f "$TEMP_FILE" "$LOG_FILE"

  whiptail --title "Success" --msgbox "Log file cleaned: $LOG_FILE" $WHIPTAIL_ROWS $WHIPTAIL_COLS
else
  whiptail --title "Cancelled" --msgbox "Log cleaning cancelled." $WHIPTAIL_ROWS $WHIPTAIL_COLS
  exit 1
fi
