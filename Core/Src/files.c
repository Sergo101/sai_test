#include "files.h"

#include "log.h"
#include "ff.h"

#include <string.h>

#define FILENAME_EXTENSION  ".WAV"
#define FILENAME_EXTENSION_SIZE  (sizeof(FILENAME_EXTENSION) - 1)

void FindWavFiles(const char *path, Files *files) {
    files->count = 0;
    FRESULT res;

    DIR dir;
    res = f_opendir(&dir, path);
    if (res != FR_OK) {
        log_error("Error: f_opendir\n");
        return;
    }

    while (files->count < MAX_FILES_COUNT - 1) {
        FILINFO file_info;
        res = f_readdir(&dir, &file_info);
        if (res != FR_OK) {
            log_error("Error: f_readdir\n");
            return;
        }

        if (file_info.fname[0] == '\0') {
            break;
        }

        if (!(file_info.fattrib & AM_DIR)) {
            size_t len = strlen(file_info.fname);
            if (len >= FILENAME_EXTENSION_SIZE && len <= MAX_FILE_PATH_LENGTH && strcmp(&file_info.fname[len - FILENAME_EXTENSION_SIZE], FILENAME_EXTENSION) == 0) {
                printf("Found: %s\n", file_info.fname);
                strncpy(files->files[files->count], file_info.fname, MAX_FILE_PATH_LENGTH);
                files->count++;
            }
        }
    }

    f_closedir(&dir);
}
