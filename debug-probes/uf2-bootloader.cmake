
# Flash .uf2 file to RPI-RP2 disk using RP2040 bootloader.
add_custom_target(flash
    COMMENT "Copy .uf2 file to the bootloader RPI-RP2 disk"
    COMMAND powershell -Command "$rp2040Drive = (Get-WmiObject -Query 'Select * From Win32_LogicalDisk Where VolumeName=''RPI-RP2''').DeviceID; if (-not $rp2040Drive) { Write-Error 'RPI-RP2 disk not found. Reset the RP2040 board holding the BOOT button'; exit 1; }; Copy-Item -Path ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.uf2 -Destination $rp2040Drive"
    VERBATIM USES_TERMINAL
)
