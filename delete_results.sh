#/usr/bin/bash
if [ -z "$(ls -A datasave/)" ]; then
   echo "No data to delete"
   exit
fi

if whiptail --yesno --defaultno "About to delete all results data. Are you REALLY sure you want to continue? Think really hard before answering yes.\n\nPreview of first 10 files:\n`ls -1 datasave | head -10`" 20 60 --yes-button "Delete results" --no-button "Cancel"  ;then
    echo "Deleting results."
    rm datasave/*
else
    echo "Quitting, as you selected to not delete results"
fi
