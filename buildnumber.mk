# Create an auto-incrementing build number.
# see https://www.linuxjournal.com/content/add-auto-incrementing-build-number-your-build-process

BUILD_NUMBER_LDFLAGS  = -Xlinker --defsym -Xlinker __BUILD_UTIME=$$(date +'%s')
BUILD_NUMBER_LDFLAGS += -Xlinker --defsym -Xlinker __BUILD_NUMBER=$$(cat $(BUILD_NUMBER_FILE))

# Build number file.  Increment if any object file changes.
$(BUILD_NUMBER_FILE): $(foreach libcomp,$(COMPONENT_LIBRARIES),$(BUILD_DIR_BASE)/$(libcomp)/lib$(libcomp).a) $(COMPONENT_PROJECT_VARS)
	@if ! test -f $(BUILD_NUMBER_FILE); then echo 0 > $(BUILD_NUMBER_FILE); fi
	@echo $$(($$(cat $(BUILD_NUMBER_FILE)) + 1)) > $(BUILD_NUMBER_FILE)
	@echo build number: $$(cat $(BUILD_NUMBER_FILE))
