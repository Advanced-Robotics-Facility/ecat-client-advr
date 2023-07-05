function onAfterCodeGen(hCS, buildInfo)
%This function is for internal use only. It may be removed in the future.

%ONAFTERCODEGEN Hook point for after code generation

%   Copyright 2014-2020 The MathWorks, Inc.

    data = codertarget.data.getData(hCS);
    modelName = buildInfo.ModelName;
    % Set the Compiler requirements to support threads for C++11 (and above)  
    buildInfo.Settings.setCompilerRequirements('supportCPPThread', true);       
    % Open new stage in diagnostic viewer and close it when function exits or
    % if an error occurs
    archiveStage = sldiagviewer.createStage(message('ros:slros:deploy:BuildArchiveStage').getString, 'ModelName', modelName);
    stageCleanup = onCleanup(@() delete(archiveStage));

    % The zip file will be put in the start dir, the same as the final
    % executable or dll.
    sDir = getSourcePaths(buildInfo, true, {'StartDir'});
    if isempty(sDir)
        sDir = {pwd};
    end
    archiveName = [modelName, '.tgz'];
    archive = fullfile(sDir{1}, archiveName);

    % If model did not change and archive exists, there is no need to
    % regenerate it. Return right away.
    lIsNewCode = ros.codertarget.internal.hasModelChanged;
    if ~lIsNewCode && exist(archive, 'file')
        disp(message('ros:slros:deploy:ArchiveUpToDate', modelName, archive).getString);
        return;
    end

    disp(message('ros:slros:deploy:CreateArchiveFile', modelName).getString);
    disp(ros.slros.internal.diag.DeviceDiagnostics.StepSeparator);

    % Add node related build artifacts to buildInfo
    nodeInfo = ros.slros.internal.cgen.postCodeGenHook(hCS, buildInfo);

    if ~ros.codertarget.internal.Util.isTopLevelModel(buildInfo)
        % Update buildInfo only for the referenced models

        extList = {'.c' '.C' '.cpp' '.CPP' '.s' '.S'};
        incExtList = {'.h' '.H', '.hpp'};

        updateFilePathsAndExtensions(buildInfo, extList, incExtList);
    end

    % ignoreParseError converts parsing errors from findIncludeFiles into warnings
    findIncludeFiles(buildInfo, ...
                     'extensions', {'*.h' '*.hpp'}, ...
                     'ignoreParseError', true);

    % Temporary fix for C++ code generation
    if isequal(get_param(modelName, 'TargetLang'), 'C++')
        loc_removeSources(buildInfo, {'ert_main.c', ...
                'ert_main.cpp', ...
                'rt_cppclass_main.cpp', ...
                'linuxinitialize.cpp'});
    end

    % Replace the define '-DRT' with '-DRT=RT'. This define clashes with a
    % definition in BOOST math library
    defToReplace.Name = 'RT';
    defToReplace.ReplaceWith = 'RT=RT';
    loc_replaceDefines(buildInfo, defToReplace);

    %% Replace host-specific UDP block files with target-specific ones
    fileToFind = fullfile('$(MATLAB_ROOT)','toolbox','shared','spc','src_ml','extern','src','DAHostLib_Network.c');
    found = loc_findInBuildInfoSrc(buildInfo,fileToFind);
    if ~isempty(found)
        sourceFolder = ros.slros.internal.cgen.Constants.PredefinedCode.Location;
        loc_addUDPBlocksToBuildInfo(buildInfo, sourceFolder);
    end

    bDir = getSourcePaths(buildInfo, true, {'BuildDir'});
    if isempty(bDir)
        bDir = {pwd};
    end

    % Create CMakeLists.txt file for our node
    cmakeVersion = '2.8.3';
    packageDep = {'roscpp', 'std_msgs'};
    packageDep = unique([packageDep nodeInfo.nodeDependencies]);
    systemDep = {};
    msgFiles = {};
    msgDep = {};

    isTopLevelModel = ros.codertarget.internal.Util.isTopLevelModel(buildInfo);
    if isTopLevelModel
        % Find names for all referenced models.
        [~, refNodeInfo]= ros.codertarget.internal.Util.uniqueModelRefNames(buildInfo);
        % Include package dependencies needed for model references to top
        % level model
        for i = 1:numel(refNodeInfo)
            thisNodeInfo = refNodeInfo{i};
            packageDep = unique([packageDep thisNodeInfo.nodeDependencies]);
        end
    end
    
    % CMakelists.txt and package.xml both have to use the same ROS package name
    % (required to be all lowercase and start with a non-numeric letter).

    packageName = ros.codertarget.internal.Util.modelNameToValidPackageName(modelName);
    writeCMakeListsTxt(buildInfo, cmakeVersion, packageName, packageDep, systemDep, msgFiles, msgDep);

    % Create package.xml file for our node
    writePackageXml(...
        modelName, ...
        packageName, ...
        data.Packaging.MaintainerName, ...
        data.Packaging.MaintainerEmail, ...
        data.Packaging.License, ...
        data.Packaging.Version, ...
        packageDep, ...
        packageDep);

    % Get full list of files that need to be archived for this model
    filePaths = getFullFileList(buildInfo);

    % Exclude all system includes, since they are expected to exist on the
    % target and cannot be packaged in our TAR file.
    isSysInclude = string(filePaths).startsWith('<');
    if any(isSysInclude)
        filePaths = filePaths(~isSysInclude);
    end

    % Add CATKIN artifacts CMakeLists.txt and package.xml to the archive
    allFiles = [filePaths, ...
                fullfile(bDir{1}, 'CMakeLists.txt'), ...
                fullfile(bDir{1}, 'package.xml'), ...
               ];

    % Add headers for the shared utilities in rtwshared.lib. This is required,
    % since for some models in the model reference hierarchy, the build
    % includes C++ files, but not the corresponding headers. To be on the safe
    % side, deploy all shared headers as part of the model
    sharedutilsdir = ros.codertarget.internal.Util.sharedUtilsDir(buildInfo, true);
    if ~isempty(sharedutilsdir)
        sharedHeaderInfo = dir(fullfile(sharedutilsdir, '*.h'));
        sharedHeaders = {sharedHeaderInfo.name};
        allFiles = [allFiles, fullfile(sharedutilsdir, sharedHeaders)];
        allFiles = unique(allFiles);
    end


    % Exclude files from all model references. Since each model reference will
    % create its own archive, the top-level model does not need to include
    % files from any of the references.
    for i = 1:numel(buildInfo.ModelRefs)
        mref = buildInfo.ModelRefs(i);

        % Get all source and header files for this model reference with no macro resolution
        sourceFiles = mref.getSourceFiles(buildInfo, false);
        headerFiles = mref.getIncludeFiles(buildInfo, false);

        % Find the indices of all the files that are part of the model reference
        [~,mrefName] = fileparts(mref.Path);
        mrefPath = string(fullfile('slprj', 'ert', mrefName));
        mrefSourceIdx = string(sourceFiles).contains(mrefPath);
        mrefHeaderIdx = string(headerFiles).contains(mrefPath);

        % Use the indices to find the fully resolved file names
        resolvedSourceFiles = mref.getSourceFiles(buildInfo, true);
        resolvedHeaderFiles = mref.getIncludeFiles(buildInfo, true);
        mrefFiles = [...
            resolvedSourceFiles(mrefSourceIdx) ...
            resolvedHeaderFiles(mrefHeaderIdx) ...
                    ];

        % Exclude these files from the archive
        allFiles = setdiff(allFiles, mrefFiles);
    end

    % Create archive
    tar(archive, allFiles);

    disp(message('ros:slros:deploy:ArchiveTargetFile', archive).getString);

    % Copy build_model.sh to the same directory as the archive file
    scriptName = 'build_ros_model.sh';
    targetScript = fullfile(sDir{1}, scriptName);
    scriptLoc = ros.slros.internal.cgen.Constants.PredefinedCode.Location;
    copyfile(fullfile(scriptLoc, scriptName), targetScript, 'f');

    disp(message('ros:slros:deploy:ShellScriptTargetFile', targetScript).getString);
    disp(ros.slros.internal.diag.DeviceDiagnostics.StepSeparator);
    disp(message('ros:slros:deploy:CopyFilesToTarget').getString);

end

%--------------------------------------------------------------------------
% Internal functions
%--------------------------------------------------------------------------
function writeCMakeListsTxt(buildInfo, cmakeVersion, packageName, packageDep, systemDep, msgFiles, msgDep)
    hs = StringWriter;

    isTopLevelModel = ros.codertarget.internal.Util.isTopLevelModel(buildInfo);

    % Find names for all referenced models.
    modelRefNames = ros.codertarget.internal.Util.uniqueModelRefNames(buildInfo);

    % 1. Required CMake Version (cmake_minimum_required)
    % 2. Package Name (project())
    hs.addcr(sprintf('cmake_minimum_required(VERSION %s)', cmakeVersion));
    hs.addcr(sprintf('project(%s)', packageName));
    hs.addcr('set(CMAKE_VERBOSE_MAKEFILE OFF)');
    hs.addcr();

    %#  C++ Standard
    hs.addcr('# C++ standard');
    hs.addcr('set(CMAKE_CXX_STANDARD 17)')
    hs.addcr('set(CMAKE_CXX_STANDARD_REQUIRED ON)')

    if isTopLevelModel
        % Propagate project name into all nested CMake projects
        hs.addcr('## Make top-level project name available in all nested projects.');
        hs.addcr('set(MW_TOP_LEVEL_PROJECT_NAME ${PROJECT_NAME})');
        hs.addcr();

        % Build subdirectories for model references
        for i = 1:numel(modelRefNames)
            hs.addcr('add_subdirectory(%s)', lower(modelRefNames{i}));
        end
    end
    hs.addcr();

    %3. Find other CMake/Catkin packages needed for build (find_package())
    if ~isempty(packageDep)
        hs.addcr('## Find catkin macros and libraries');
        hs.addcr('## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)');
        hs.addcr('## is used, also find other catkin packages');
        hs.addcr('find_package(catkin REQUIRED COMPONENTS');
        for k = 1:numel(packageDep)
            hs.addcr(sprintf('%s', packageDep{k}));
        end
    else
        hs.addcr('## Find catkin macros and libraries');
        hs.addcr('## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)');
        hs.addcr('## is used, also find other catkin packages');
        hs.addcr('find_package(catkin REQUIRED');
    end
    %% This is how you add BOOST components
    %hs.addcr('find_package(Boost REQUIRED COMPONENTS math)');
    hs.addcr(')');
    hs.addcr();

    % 4. Message/Service/Action Generators (add_message_files(), add_service_files(), add_action_files())
    if ~isempty(msgFiles)
        hs.addcr('## System dependencies are found with CMake''s conventions');
        hs.addcr('# find_package(Boost REQUIRED COMPONENTS system)');
        hs.addcr();
        hs.addcr('################################################');
        hs.addcr('## Declare ROS messages, services and actions ##');
        hs.addcr('################################################');
        hs.addcr('add_message_files(');
        hs.addcr('#   FILES');
        for k = 1:numel(msgFiles)
            hs.addcr(sprintf('%s', msgFiles{k}));
        end
        hs.addcr(')');
        hs.addcr();
    end

    %#  Find BlockFactory
    hs.addcr('# Find BlockFactory');
    hs.addcr('find_package(BlockFactory REQUIRED COMPONENTS Core SimulinkCoder)');
    hs.addcr();

    found_rt_plugin = false;
    plugin_expr = '_rt_plugin';

    % Get all C++ source files (with no paths and no macro resolution)
    % Add them to compile list for executable
    srcFiles = getSourceFiles(buildInfo, false, false);

    for k = 1:numel(srcFiles)
        [~, name, ext] = fileparts(srcFiles{k});
	plugin_index = regexp(name,plugin_expr);
        if(~isempty(plugin_index)) 
	    found_rt_plugin = true;
            break;
	end
    end

    if found_rt_plugin
	    %#  Find XBot2
	    hs.addcr('# Find XBot2');
	    hs.addcr('find_package(xbot2 REQUIRED)');
	    hs.addcr();

	    %#  Find XBotBlock
	    hs.addcr('# Find XBotBlock');
	    hs.addcr('find_package(XBotBlock REQUIRED)');
	    hs.addcr();

	    %#  Add internal variable for RT xbot2 plugin
	    hs.addcr('option(XBOT2_ENABLE_XENO OFF "Compile against xenomai")');
    end

    %5. Invoke message/service/action generation (generate_messages())
    if ~isempty(msgDep) && 0
        hs.addcr('## Generate added messages and services with any dependencies listed here');
        hs.addcr('generate_messages(');
        hs.addcr('   DEPENDENCIES');
        for k = 1:numel(msgDep)
            hs.addcr(sprintf('   %s', msgDep{k}));
        end
        hs.addcr(')');
        hs.addcr();
    end

    % Add project specific definitions
    def = buildInfo.getDefines;
    if ~isempty(def)
        hs.addcr('add_definitions(')
        for k = 1:numel(def)
            hs.add(def{k});
            hs.add(' ');
        end
        hs.addcr(')');
        hs.addcr();
    end

     if found_rt_plugin
	     % Find xenoami definition and set on/off the internal variable
	     hs.addcr('set(XBOT2_ENABLE_XENO OFF CACHE BOOL "Disable real-time features" FORCE)')
	     hs.addcr('get_directory_property(COMP_DEF COMPILE_DEFINITIONS)')
	     hs.addcr('foreach( d ${COMP_DEF} )')
	     hs.addcr('    if (${d} STREQUAL "XBOT2_ENABLE_XENO=ON")')
	     hs.addcr('       set(XBOT2_ENABLE_XENO ON CACHE BOOL "Enable real-time features" FORCE)')
	     hs.addcr('       continue()')
	     hs.addcr('    endif()')
	     hs.addcr('endforeach()')
    end

    %6. Specify package build info export (catkin_package())
    hs.addcr('## The catkin_package macro generates cmake config files for your package');
    hs.addcr('## Declare things to be passed to dependent projects');
    hs.addcr('## INCLUDE_DIRS: uncomment this if you package contains header files');
    hs.addcr('## LIBRARIES: libraries you create in this project that dependent projects also need');
    hs.addcr('## CATKIN_DEPENDS: catkin_packages dependent projects also need');
    hs.addcr('## DEPENDS: system dependencies of this project that dependent projects also need');
    hs.addcr('catkin_package(');
    libraries = {};
    includeDirs = {};
    %#  INCLUDE_DIRS include
    if ~isempty(includeDirs)
        hs.add('   INCLUDE_DIRS ');
        for k = 1:numel(includeDirs)
            hs.add(includeDirs{k});
            hs.add(' ');
        end
        hs.addcr();
    end
    %#  LIBRARIES beginner_tutorials
    if ~isempty(libraries)
        hs.add('   LIBRARIES ');
        for k = 1:numel(libraries)
            hs.add(libraries{k});
            hs.add(' ');
        end
        hs.addcr();
    end
    %#  CATKIN_DEPENDS roscpp rospy std_msgs
    if ~isempty(packageDep)
        hs.add('   CATKIN_DEPENDS ');
        for k = 1:numel(packageDep)
            hs.add(packageDep{k});
            hs.add(' ');
        end
        hs.addcr();
    end
    %#  DEPENDS system_lib
    if ~isempty(systemDep)
        hs.add('   DEPENDS ');
        for k = 1:numel(systemDep)
            hs.add(systemDep{k});
            hs.add(' ');
        end
        hs.addcr();
    end
    hs.addcr(')');
    hs.addcr();

    % buildInfo.getLinkFlags
    %
    % buildInfo.getIncludePaths(true)

    % 7. Libraries/Executables to build (add_library()/add_executable()/target_link_libraries())
    % The PROJECT_SOURCE_DIRECTORY is listed explicitly to pick up header
    % files that are included using angle brackets
    hs.addcr('###########');
    hs.addcr('## Build ##');
    hs.addcr('###########');
    hs.addcr();
    hs.addcr('## Specify additional locations of header files');
    hs.addcr('## Your package locations should be listed before other locations');
    hs.addcr('# include_directories(include)');
    hs.addcr('include_directories(');
    hs.addcr('  include');
    hs.addcr('  ${PROJECT_SOURCE_DIR}');
    hs.addcr('  ${Boost_INCLUDE_DIRS}');
    hs.addcr('  ${catkin_INCLUDE_DIRS}');

    % Add include directories for model references
    currentSourceDir = '  ${CMAKE_CURRENT_SOURCE_DIR}';
    for i = 1:numel(modelRefNames)
        packageDirName = ros.codertarget.internal.Util.modelNameToValidPackageName(modelRefNames{i});
        % Add two additional include directories for each model reference:
        % - relative to the current directory (used for top-level model)
        % - up one level (used for nested model references)
        hs.addcr('%s/%s', currentSourceDir, packageDirName);
        hs.addcr('%s/../%s', currentSourceDir, packageDirName);
    end

    hs.addcr(')');
    hs.addcr();

    hs.addcr('## Get list of .c files in project source directory');
    hs.addcr('file(GLOB ${PROJECT_NAME}_C_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/*.c)');
    hs.addcr('## Get list of .cpp files in project source directory');
    hs.addcr('file(GLOB ${PROJECT_NAME}_CPP_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/*.cpp)');

    hs.addcr();

    %exlude rt plugin for nrt ros node	
    hs.addcr('list(FILTER ${PROJECT_NAME}_CPP_SOURCES EXCLUDE REGEX ".*_rt_plugin\\.cpp$")');

    % Declare executable or library to build
    if isTopLevelModel
        % Build an executable for the top-level model
        buildTargetName = '${PROJECT_NAME}_node';
        hs.addcr('## Declare executable');
        hs.addcr(['add_executable(' buildTargetName]);
    else
        % Build a shared library for all referenced models
        % Use a unique target name for the library, since CMake enforces 
        % the uniqueness across multiple ROS packages in the same Catkin 
        % workspace. See https://cmake.org/cmake/help/v3.0/policy/CMP0002.html.
        % Pick a combination of the top-level model name and the
        % referenced model name. The referenced model is shared in
        % the whole model reference hierarchy, so using the top-level model
        % name ensures uniqueness
        buildTargetName = '${MW_TOP_LEVEL_PROJECT_NAME}_${PROJECT_NAME}';
        hs.addcr('## Declare library');
        hs.addcr(['add_library(' buildTargetName ' SHARED']);
    end

    % Add C++ sources to compilation list for executable / shared library
    for k = 1:numel(srcFiles)
        [~, name, ext] = fileparts(srcFiles{k});
        % avoid to compile rt plugin
        plugin_index = regexp(name,plugin_expr);
        if(isempty(plugin_index))
	   hs.addcr(sprintf('   %s', [name, ext]));
	end
    end

    % Add shared source files to the compilation list as well
    % This is used for source files that are shared among top-level model and
    % model references.
    sharedcpp = findLinkObject(buildInfo,'rtwshared.a');
    if ~isempty(sharedcpp)
        sharedfiles = getSourceFiles(sharedcpp,true,true);
        for k = 1:numel(sharedfiles)
            [~, name, ext] = fileparts(sharedfiles{k});
            hs.addcr(sprintf('   %s', [name, ext]));
        end
    end

    % Also add all plain C and C++ sources (e.g. if code generation happens for blocks
    % from other toolboxes.
    hs.addcr('   ${${PROJECT_NAME}_C_SOURCES}');
    hs.addcr('   ${${PROJECT_NAME}_CPP_SOURCES}');
    hs.addcr(')');
    hs.addcr();

    if found_rt_plugin

	    buildLibName = '${PROJECT_NAME}_plugin';
	    hs.addcr('## Declare library');
	    hs.addcr(['add_xbot2_plugin(' buildLibName ]);

	    % exclude main, ros and PIL communication
	    main_filename = 'main';
	    ros_filename = ["rosnodeinterface","slros_busmsg_conversion","slros_initialize","slros_generic_param"];
	    pil_filename = ["ext_svr","updown","ext_work","rtiostream_utils","rtiostream_interface","rtiostream_tcpip"];

	    % Add C++ sources to compilation list for executable / shared library
	    for k = 1:numel(srcFiles)
		[~, name, ext] = fileparts(srcFiles{k});
		
		main_index = regexp(name,main_filename);

		if(isempty(main_index)) 
		    for i=1:length(ros_filename) 
		      rosnode_index = regexp(name,ros_filename(i));
		      if(~isempty(rosnode_index))
			break;
		      end
		    end
		end

		if(isempty(rosnode_index)) 
		    for i=1:length(pil_filename) 
		      pil_index = regexp(name,pil_filename(i));
		      if(~isempty(pil_index))
			break;
		      end
		    end
		end
		
		if(isempty(main_index) && isempty(rosnode_index) && isempty(pil_index))
		   hs.addcr(sprintf('   %s', [name, ext]));
		end
	    end
	    hs.addcr(')');
	    hs.addcr();
    end

    % Add dependencies for custom messages
    hs.addcr('## Add cmake target dependencies of the executable/library');
    hs.addcr('## as an example, message headers may need to be generated before nodes');
    hs.addcr(['add_dependencies(' buildTargetName ' ${catkin_EXPORTED_TARGETS})']);
    hs.addcr();

    % Add rules for custom libraries (added through "custom code" pane in code
    % generation settings).
    allLinkObjects = getLinkObjects(buildInfo);

    % Use only link object that are LinkOnly, i.e. where no compilation or
    % pre-link processing should be done.
    isCustomObj = arrayfun(@(linkObj) linkObj.LinkOnly, allLinkObjects);
    customLinkObjects = allLinkObjects(isCustomObj);

    customLibNames = cell(numel(customLinkObjects),1);
    for k = 1:numel(customLinkObjects)
        % Ignore rtwshared[.lib,.a] as a custom link object. Compile the
        % shared utility sources as part of the ROS build project.
        if ismember(customLinkObjects(k).Name,{'rtwshared','rtwshared.lib','rtwshared.a'})
            continue;
        end

        % Declare custom libraries as IMPORTEDs, so that we have the full
        % absolute path to the libraries during the CMake execution. This ensures
        % that even library names that do not begin with lib* are picked up
        % correctly. See http://stackoverflow.com/questions/33165270/force-cmake-to-use-the-full-library-path
        customLibNames{k,1} = ['slcoder_custom_lib_' num2str(k)];
        hs.addcr(['## Rule for imported library ' customLinkObjects(k).Name]);
        hs.addcr(['add_library(' customLibNames{k,1} ' UNKNOWN IMPORTED)']);

        % All custom library files will be in the flat PROJECT_SOURCE_DIR folder
        hs.addcr(['set_property(TARGET ' customLibNames{k,1} ' PROPERTY IMPORTED_LOCATION "${PROJECT_SOURCE_DIR}/' customLinkObjects(k).Name '")']);
        hs.addcr();
    end

    % Add link libraries
    hs.addcr('## Specify libraries to link a library or executable target against');
    hs.addcr(['target_link_libraries(' buildTargetName]);

    % Only for the top-level model, link against all libraries created for referenced models
    if isTopLevelModel
        for i = 1:numel(modelRefNames)
            actualLibName = ros.codertarget.internal.Util.modelNameToValidPackageName(modelRefNames{i});
            % Link against that library
            % The shared library name is using a combination of top-level and referenced model names.
            hs.addcr(sprintf('${MW_TOP_LEVEL_PROJECT_NAME}_%s', actualLibName));
        end
    end

    % Add standard link targets for Catkin and Boost
    hs.addcr('   ${catkin_LIBRARIES}');
    hs.addcr('   ${Boost_LIBRARIES}');

    % Add dependent libraries such as librt, libpthread, etc.
    systemLibs = getSystemLibraries(buildInfo);
    for k = 1:numel(systemLibs)
        hs.addcr(['   ', systemLibs{k}]);
    end

    % Also link against any custom libraries
    for k = 1:numel(customLinkObjects)
        hs.addcr(['   ', customLibNames{k,1}]);
    end
    % Add platform specific dynamic library loader flag, e.g. '-ldl' on
    % Unix.
    hs.addcr('   ${CMAKE_DL_LIBS}');
    hs.addcr(')');
    hs.addcr();

    % Put all shared libraries (for model references) in same folder as node
    % binary. By default, all shared libraries are placed in devel/lib, but
    % that could lead to potential conflicts if multiple deployed nodes build the same
    % referenced model, but with a different version.
    %
    % See above for setting the MW_TOP_LEVEL_PROJECT_NAME variable in the top-level model.
    if ~isTopLevelModel
        hs.addcr('## Generate the library in the same folder as the node binary to avoid version conflicts between multiple nodes.');
        hs.addcr(['set_target_properties(' buildTargetName ' PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/lib/${MW_TOP_LEVEL_PROJECT_NAME}")']);
        
        % Set the output name of the library based on just the model name, not the combination
        % of top-level and referenced model name that we use for buildTargetName.
        hs.addcr(['set_target_properties(' buildTargetName ' PROPERTIES OUTPUT_NAME ${PROJECT_NAME})']);
        hs.addcr();
    end

    % The compile flags
    compileFlags = strtrim(buildInfo.getCompileFlags);
    if ~isempty(compileFlags)
        hs.add(['set_target_properties(' buildTargetName ' PROPERTIES COMPILE_FLAGS "']);
        for k = 1:numel(compileFlags)
            if ~strcmp(compileFlags{k}, '-fpermissive')
                % -fpermissive is needed to build the C++ POSIX scheduler.
                % This is only needed for C++ and is handled by the
                % CMAKE_CXX_FLAGS statement below
                hs.add([compileFlags{k}, ' ']);
            end
        end
        hs.add('"');
        hs.addcr(')');
    end
    hs.addcr('SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -fpermissive" )');
    % stdbool.h is needed for C sources that use true/false keywords
    hs.addcr('SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -include stdbool.h" )');

    % Link and Include BlockFactory libraries
    hs.addcr('# Link the node with the Core component containing the core classes');
    hs.addcr('target_link_libraries(${PROJECT_NAME}_node PRIVATE BlockFactory::Core BlockFactory::SimulinkCoder)');

    hs.addcr('# Setup the include directories');
    hs.addcr('target_include_directories(${PROJECT_NAME}_node PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)');

    if found_rt_plugin
	    % Link and Include BlockFactory libraries
	    hs.addcr('# Link the plugin with the Core component containing the core classes');
	    hs.addcr('target_link_libraries(${PROJECT_NAME}_plugin PRIVATE BlockFactory::Core BlockFactory::SimulinkCoder XBotBlock::XBotBlock_Utils)');

	    hs.addcr('# Setup the include directories');
	    hs.addcr('target_include_directories(${PROJECT_NAME}_plugin PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)');
    end


    % 8. Tests to build (catkin_add_gtest())
    % if 0
    %     % 9. Install rules (install())
    %     hs.addcr('install(TARGETS ${PROJECT_NAME}_node');
    %     hs.addcr('  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}');
    %     hs.addcr('  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}');
    %     hs.addcr('  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}');
    %     hs.addcr(')');
    %     hs.addcr();
    % end

    hs.write('CMakeLists.txt');
end

%--------------------------------------------------------------------------
function writePackageXml(modelName, packageName, maintainerName, maintainerEmail, licenseType, version, buildDep, runDep)
    hs = StringWriter;

    % Create package.xml file

    hs.addcr('<package>');
    hs.addcr(sprintf('  <name>%s</name>', packageName));
    hs.addcr(sprintf('  <version>%s</version>', version));
    hs.addcr('  <description>');
    hs.addcr(sprintf('  This package is generated from %s Simulink model.', modelName));
    hs.addcr('  </description>');
    hs.addcr(sprintf('  <maintainer email="%s">%s</maintainer>', maintainerEmail, maintainerName));
    hs.addcr('  <license>%s</license>', licenseType);
    hs.addcr();
    hs.addcr('  <buildtool_depend>catkin</buildtool_depend>');
    hs.addcr();
    if ~isempty(buildDep)
        for k = 1:numel(buildDep)
            hs.addcr('  <build_depend>%s</build_depend>', buildDep{k});
        end
    end
    hs.addcr();
    if ~isempty(runDep)
        for k = 1:numel(runDep)
            hs.addcr('  <run_depend>%s</run_depend>', runDep{k});
        end
    end
    hs.addcr('</package>');
    hs.write('package.xml');
end

%--------------------------------------------------------------------------
function libs = getSystemLibraries(buildInfo)
    linkFlags = strtrim(buildInfo.getLinkFlags);
    libs = {};
    for k = 1:numel(linkFlags)
        tmp = regexp(linkFlags{k}, '-l(\S+)', 'tokens');
        if ~isempty(tmp)
            for j = 1:numel(tmp)
                libs = [libs, tmp{j}{1}]; %#ok<AGROW>
            end
        end
    end
end

%--------------------------------------------------------------------------
function loc_removeSources(buildInfo, sourcesToRemove)
    for i = 1:length(sourcesToRemove)
        srcs = buildInfo.getSourceFiles(false, false);
        found = strcmp(srcs, sourcesToRemove{i});
        buildInfo.Src.Files(found) = [];
    end
end

%--------------------------------------------------------------------------
function loc_replaceDefines(buildInfo, defToRemove)
    def = buildInfo.getDefines;
    for j = 1:numel(defToRemove)
        for k = 1:numel(def)
            if isequal(def{k}, ['-D', defToRemove(j).Name])
                buildInfo.deleteDefines(defToRemove(j).Name);
                buildInfo.addDefines(defToRemove(j).ReplaceWith);
                break;
            end
        end
    end
end

%--------------------------------------------------------------------------
function found = loc_findInBuildInfoSrc(buildInfo,filename)
    filename = strrep(filename,'$(MATLAB_ROOT)',matlabroot);
    found = [];
    for j=1:length(buildInfo.Src.Files)
        iFile = fullfile(buildInfo.Src.Files(j).Path, buildInfo.Src.Files(j).FileName);
        iFile = strrep(iFile,'$(MATLAB_ROOT)',matlabroot);
        if contains(iFile, filename)
            found = iFile;
            break;
        end
    end
end

%--------------------------------------------------------------------------
function loc_addUDPBlocksToBuildInfo(buildInfo, sourceFolder)
    filePathToAdd = sourceFolder;
    fileNameToAdd = 'linuxUDP.c';

    addSourceFiles(buildInfo,fileNameToAdd,filePathToAdd);
    addDefines(buildInfo,'_USE_TARGET_UDP_');
end

