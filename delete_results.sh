#/usr/bin/bash
if whiptail --yesno --defaultno "About to delete all results data. Are you REALLY sure you want to continue? Think really hard before answering yes." 20 60 --yes-button "Delete results" --no-button "Cancel"  ;then
    echo "Deleting results."
    rm *.result
else
    echo "Quitting, as you selected to not delete results"
fi
