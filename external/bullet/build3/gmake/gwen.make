# GNU Make project makefile autogenerated by Premake
ifndef config
  config=release64
endif

ifndef verbose
  SILENT = @
endif

ifndef CC
  CC = gcc
endif

ifndef CXX
  CXX = g++
endif

ifndef AR
  AR = ar
endif

ifndef RESCOMP
  ifdef WINDRES
    RESCOMP = $(WINDRES)
  else
    RESCOMP = windres
  endif
endif

ifeq ($(config),release64)
  OBJDIR     = obj/x64/Release/gwen
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/libgwen_gmake_x64_release.a
  DEFINES   += -DGLEW_INIT_OPENGL11_FUNCTIONS=1 -DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1 -DGLEW_STATIC -DDYNAMIC_LOAD_X11_FUNCTIONS -DGWEN_COMPILE_STATIC -DDONT_USE_GLUT
  INCLUDES  += -I../../examples/ThirdPartyLibs/glad -I../../examples/ThirdPartyLibs/optionalX11 -I../../examples/ThirdPartyLibs/Gwen -I../../examples/ThirdPartyLibs
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -O2 -msse2 -ffast-math -m64 -fPIC
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -s -m64 -L/usr/lib64
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += -ldl -lpthread
  LDDEPS    += 
  LINKCMD    = $(AR) -rcs $(TARGET) $(OBJECTS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),debug64)
  OBJDIR     = obj/x64/Debug/gwen
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/libgwen_gmake_x64_debug.a
  DEFINES   += -D_DEBUG=1 -DGLEW_INIT_OPENGL11_FUNCTIONS=1 -DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1 -DGLEW_STATIC -DDYNAMIC_LOAD_X11_FUNCTIONS -DGWEN_COMPILE_STATIC -DDONT_USE_GLUT
  INCLUDES  += -I../../examples/ThirdPartyLibs/glad -I../../examples/ThirdPartyLibs/optionalX11 -I../../examples/ThirdPartyLibs/Gwen -I../../examples/ThirdPartyLibs
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -g -ffast-math -m64 -fPIC
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -m64 -L/usr/lib64
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += -ldl -lpthread
  LDDEPS    += 
  LINKCMD    = $(AR) -rcs $(TARGET) $(OBJECTS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

OBJECTS := \
	$(OBJDIR)/glx.o \
	$(OBJDIR)/gl.o \
	$(OBJDIR)/events.o \
	$(OBJDIR)/DragAndDrop.o \
	$(OBJDIR)/Anim.o \
	$(OBJDIR)/Hook.o \
	$(OBJDIR)/BaseRender.o \
	$(OBJDIR)/Gwen.o \
	$(OBJDIR)/Skin.o \
	$(OBJDIR)/inputhandler.o \
	$(OBJDIR)/ToolTip.o \
	$(OBJDIR)/Utility.o \
	$(OBJDIR)/TreeControl.o \
	$(OBJDIR)/RadioButtonController.o \
	$(OBJDIR)/TextBoxNumeric.o \
	$(OBJDIR)/Base.o \
	$(OBJDIR)/PanelListPanel.o \
	$(OBJDIR)/Text.o \
	$(OBJDIR)/HorizontalSlider.o \
	$(OBJDIR)/GroupBox.o \
	$(OBJDIR)/Slider.o \
	$(OBJDIR)/NumericUpDown.o \
	$(OBJDIR)/Dragger.o \
	$(OBJDIR)/TabButton.o \
	$(OBJDIR)/RichLabel.o \
	$(OBJDIR)/Menu.o \
	$(OBJDIR)/MenuStrip.o \
	$(OBJDIR)/ResizableControl.o \
	$(OBJDIR)/VerticalSlider.o \
	$(OBJDIR)/ImagePanel.o \
	$(OBJDIR)/HSVColorPicker.o \
	$(OBJDIR)/ProgressBar.o \
	$(OBJDIR)/ListBox.o \
	$(OBJDIR)/ScrollBarBar.o \
	$(OBJDIR)/ScrollBarButton.o \
	$(OBJDIR)/ScrollControl.o \
	$(OBJDIR)/DockedTabControl.o \
	$(OBJDIR)/ColorPicker.o \
	$(OBJDIR)/CrossSplitter.o \
	$(OBJDIR)/Resizer.o \
	$(OBJDIR)/TabControl.o \
	$(OBJDIR)/TextBox.o \
	$(OBJDIR)/LabelClickable.o \
	$(OBJDIR)/CheckBox.o \
	$(OBJDIR)/SplitterBar.o \
	$(OBJDIR)/VerticalScrollBar.o \
	$(OBJDIR)/Canvas.o \
	$(OBJDIR)/ComboBox.o \
	$(OBJDIR)/Properties.o \
	$(OBJDIR)/ScrollBar.o \
	$(OBJDIR)/ColorControls.o \
	$(OBJDIR)/RadioButton.o \
	$(OBJDIR)/Button.o \
	$(OBJDIR)/Label.o \
	$(OBJDIR)/HorizontalScrollBar.o \
	$(OBJDIR)/TreeNode.o \
	$(OBJDIR)/MenuItem.o \
	$(OBJDIR)/TabStrip.o \
	$(OBJDIR)/WindowControl.o \
	$(OBJDIR)/DockBase.o \
	$(OBJDIR)/FileOpen.o \
	$(OBJDIR)/FileSave.o \
	$(OBJDIR)/Query.o \
	$(OBJDIR)/Windows.o \
	$(OBJDIR)/Null.o \
	$(OBJDIR)/OpenGL_DebugFont.o \

RESOURCES := \

SHELLTYPE := msdos
ifeq (,$(ComSpec)$(COMSPEC))
  SHELLTYPE := posix
endif
ifeq (/bin,$(findstring /bin,$(SHELL)))
  SHELLTYPE := posix
endif

.PHONY: clean prebuild prelink

all: $(TARGETDIR) $(OBJDIR) prebuild prelink $(TARGET)
	@:

$(TARGET): $(GCH) $(OBJECTS) $(LDDEPS) $(RESOURCES)
	@echo Linking gwen
	$(SILENT) $(LINKCMD)
	$(POSTBUILDCMDS)

$(TARGETDIR):
	@echo Creating $(TARGETDIR)
ifeq (posix,$(SHELLTYPE))
	$(SILENT) mkdir -p $(TARGETDIR)
else
	$(SILENT) mkdir $(subst /,\\,$(TARGETDIR))
endif

$(OBJDIR):
	@echo Creating $(OBJDIR)
ifeq (posix,$(SHELLTYPE))
	$(SILENT) mkdir -p $(OBJDIR)
else
	$(SILENT) mkdir $(subst /,\\,$(OBJDIR))
endif

clean:
	@echo Cleaning gwen
ifeq (posix,$(SHELLTYPE))
	$(SILENT) rm -f  $(TARGET)
	$(SILENT) rm -rf $(OBJDIR)
else
	$(SILENT) if exist $(subst /,\\,$(TARGET)) del $(subst /,\\,$(TARGET))
	$(SILENT) if exist $(subst /,\\,$(OBJDIR)) rmdir /s /q $(subst /,\\,$(OBJDIR))
endif

prebuild:
	$(PREBUILDCMDS)

prelink:
	$(PRELINKCMDS)

ifneq (,$(PCH))
$(GCH): $(PCH)
	@echo $(notdir $<)
ifeq (posix,$(SHELLTYPE))
	-$(SILENT) cp $< $(OBJDIR)
else
	$(SILENT) xcopy /D /Y /Q "$(subst /,\,$<)" "$(subst /,\,$(OBJDIR))" 1>nul
endif
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
endif

$(OBJDIR)/glx.o: ../../examples/ThirdPartyLibs/glad/glx.c
	@echo $(notdir $<)
	$(SILENT) $(CC) $(CFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/gl.o: ../../examples/ThirdPartyLibs/glad/gl.c
	@echo $(notdir $<)
	$(SILENT) $(CC) $(CFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/events.o: ../../examples/ThirdPartyLibs/Gwen/events.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/DragAndDrop.o: ../../examples/ThirdPartyLibs/Gwen/DragAndDrop.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Anim.o: ../../examples/ThirdPartyLibs/Gwen/Anim.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Hook.o: ../../examples/ThirdPartyLibs/Gwen/Hook.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/BaseRender.o: ../../examples/ThirdPartyLibs/Gwen/BaseRender.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Gwen.o: ../../examples/ThirdPartyLibs/Gwen/Gwen.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Skin.o: ../../examples/ThirdPartyLibs/Gwen/Skin.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/inputhandler.o: ../../examples/ThirdPartyLibs/Gwen/inputhandler.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/ToolTip.o: ../../examples/ThirdPartyLibs/Gwen/ToolTip.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Utility.o: ../../examples/ThirdPartyLibs/Gwen/Utility.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/TreeControl.o: ../../examples/ThirdPartyLibs/Gwen/Controls/TreeControl.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/RadioButtonController.o: ../../examples/ThirdPartyLibs/Gwen/Controls/RadioButtonController.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/TextBoxNumeric.o: ../../examples/ThirdPartyLibs/Gwen/Controls/TextBoxNumeric.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Base.o: ../../examples/ThirdPartyLibs/Gwen/Controls/Base.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/PanelListPanel.o: ../../examples/ThirdPartyLibs/Gwen/Controls/PanelListPanel.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Text.o: ../../examples/ThirdPartyLibs/Gwen/Controls/Text.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/HorizontalSlider.o: ../../examples/ThirdPartyLibs/Gwen/Controls/HorizontalSlider.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/GroupBox.o: ../../examples/ThirdPartyLibs/Gwen/Controls/GroupBox.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Slider.o: ../../examples/ThirdPartyLibs/Gwen/Controls/Slider.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/NumericUpDown.o: ../../examples/ThirdPartyLibs/Gwen/Controls/NumericUpDown.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Dragger.o: ../../examples/ThirdPartyLibs/Gwen/Controls/Dragger.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/TabButton.o: ../../examples/ThirdPartyLibs/Gwen/Controls/TabButton.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/RichLabel.o: ../../examples/ThirdPartyLibs/Gwen/Controls/RichLabel.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Menu.o: ../../examples/ThirdPartyLibs/Gwen/Controls/Menu.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/MenuStrip.o: ../../examples/ThirdPartyLibs/Gwen/Controls/MenuStrip.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/ResizableControl.o: ../../examples/ThirdPartyLibs/Gwen/Controls/ResizableControl.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/VerticalSlider.o: ../../examples/ThirdPartyLibs/Gwen/Controls/VerticalSlider.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/ImagePanel.o: ../../examples/ThirdPartyLibs/Gwen/Controls/ImagePanel.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/HSVColorPicker.o: ../../examples/ThirdPartyLibs/Gwen/Controls/HSVColorPicker.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/ProgressBar.o: ../../examples/ThirdPartyLibs/Gwen/Controls/ProgressBar.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/ListBox.o: ../../examples/ThirdPartyLibs/Gwen/Controls/ListBox.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/ScrollBarBar.o: ../../examples/ThirdPartyLibs/Gwen/Controls/ScrollBarBar.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/ScrollBarButton.o: ../../examples/ThirdPartyLibs/Gwen/Controls/ScrollBarButton.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/ScrollControl.o: ../../examples/ThirdPartyLibs/Gwen/Controls/ScrollControl.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/DockedTabControl.o: ../../examples/ThirdPartyLibs/Gwen/Controls/DockedTabControl.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/ColorPicker.o: ../../examples/ThirdPartyLibs/Gwen/Controls/ColorPicker.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/CrossSplitter.o: ../../examples/ThirdPartyLibs/Gwen/Controls/CrossSplitter.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Resizer.o: ../../examples/ThirdPartyLibs/Gwen/Controls/Resizer.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/TabControl.o: ../../examples/ThirdPartyLibs/Gwen/Controls/TabControl.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/TextBox.o: ../../examples/ThirdPartyLibs/Gwen/Controls/TextBox.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/LabelClickable.o: ../../examples/ThirdPartyLibs/Gwen/Controls/LabelClickable.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/CheckBox.o: ../../examples/ThirdPartyLibs/Gwen/Controls/CheckBox.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/SplitterBar.o: ../../examples/ThirdPartyLibs/Gwen/Controls/SplitterBar.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/VerticalScrollBar.o: ../../examples/ThirdPartyLibs/Gwen/Controls/VerticalScrollBar.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Canvas.o: ../../examples/ThirdPartyLibs/Gwen/Controls/Canvas.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/ComboBox.o: ../../examples/ThirdPartyLibs/Gwen/Controls/ComboBox.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Properties.o: ../../examples/ThirdPartyLibs/Gwen/Controls/Properties.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/ScrollBar.o: ../../examples/ThirdPartyLibs/Gwen/Controls/ScrollBar.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/ColorControls.o: ../../examples/ThirdPartyLibs/Gwen/Controls/ColorControls.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/RadioButton.o: ../../examples/ThirdPartyLibs/Gwen/Controls/RadioButton.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Button.o: ../../examples/ThirdPartyLibs/Gwen/Controls/Button.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Label.o: ../../examples/ThirdPartyLibs/Gwen/Controls/Label.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/HorizontalScrollBar.o: ../../examples/ThirdPartyLibs/Gwen/Controls/HorizontalScrollBar.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/TreeNode.o: ../../examples/ThirdPartyLibs/Gwen/Controls/TreeNode.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/MenuItem.o: ../../examples/ThirdPartyLibs/Gwen/Controls/MenuItem.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/TabStrip.o: ../../examples/ThirdPartyLibs/Gwen/Controls/TabStrip.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/WindowControl.o: ../../examples/ThirdPartyLibs/Gwen/Controls/WindowControl.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/DockBase.o: ../../examples/ThirdPartyLibs/Gwen/Controls/DockBase.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/FileOpen.o: ../../examples/ThirdPartyLibs/Gwen/Controls/Dialog/FileOpen.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/FileSave.o: ../../examples/ThirdPartyLibs/Gwen/Controls/Dialog/FileSave.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Query.o: ../../examples/ThirdPartyLibs/Gwen/Controls/Dialog/Query.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Windows.o: ../../examples/ThirdPartyLibs/Gwen/Platforms/Windows.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/Null.o: ../../examples/ThirdPartyLibs/Gwen/Platforms/Null.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
$(OBJDIR)/OpenGL_DebugFont.o: ../../examples/ThirdPartyLibs/Gwen/Renderers/OpenGL_DebugFont.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"

-include $(OBJECTS:%.o=%.d)
