################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

# app_program/*.c plus one level of subfolders (one per subsystem)
TEMP_SRCS = $(wildcard ../app_program/*.c) $(wildcard ../app_program/*/*.c)
TEMP_OBJS = $(TEMP_SRCS:.c=.o)
ifeq ($(COURSE_RUN),1)
TEMP_OBJS := $(patsubst ../app_program/subsystem5/vehicle_controller.o,../app_program/subsystem5/vehicle_controller_course.o,$(TEMP_OBJS))
endif
TEMP_DEPS = $(TEMP_OBJS:.o=.d)

OBJS += $(subst ../, ./mtkernel_3/, $(TEMP_OBJS))
C_DEPS += $(subst ../, ./mtkernel_3/, $(TEMP_DEPS))

define compile_app_object
	@echo 'Building file: $<'
	@mkdir -p $(dir $@)
	$(GCC) $(CFLAGS) $(APP_PROFILE_CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
endef

mtkernel_3/app_program/%.o: ../app_program/%.c
	$(compile_app_object)

ifeq ($(COURSE_RUN),1)
mtkernel_3/app_program/subsystem5/vehicle_controller_course.o: private APP_PROFILE_CFLAGS = -DAPP_COURSE_RUN=1
mtkernel_3/app_program/subsystem5/vehicle_controller_course.o: ../app_program/subsystem5/vehicle_controller.c
	$(compile_app_object)
endif
