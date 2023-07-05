clc
model_name = bdroot;
plugin_name = lower(model_name);
model_dir  = fileparts(get_param(bdroot,'FileName'));

model_plugin_cpp = strcat(model_dir,'/',model_name,'_rt_plugin.cpp');
model_plugin_h = strcat(model_dir,'/',model_name,'_rt_plugin.h');

fid_out_h = fopen( model_plugin_h, 'wt' );

if (fid_out_h == -1)
    disp('Impossibile aprire il file: ' + model_plugin_h);
else
    fid_in_h = fopen('model_name_plugin.h', 'r');
    if (fid_in_h == -1)
        disp('Impossibile aprire il file: model_name_plugin.h');
    else
        linea = fgetl(fid_in_h);
        while ischar(linea)
            linea = strrep(linea,'ModelName',model_name);
            fprintf(fid_out_h, '%s\n', linea);
            linea = fgetl(fid_in_h);
        end  
    end
end

fid_out_cpp = fopen( model_plugin_cpp, 'wt' );
if (fid_out_cpp == -1)
    disp('Impossibile aprire il file: ' + model_plugin_cpp);
else
    fid_in_cpp = fopen('model_name_plugin.cpp', 'r');
    if (fid_in_cpp == -1)
        disp('Impossibile aprire il file: model_name_plugin.cpp');
    else
        linea = fgetl(fid_in_cpp);
        while ischar(linea)
            linea = strrep(linea,'ModelName',model_name);
            linea = strrep(linea,'model_name',plugin_name);
            fprintf(fid_out_cpp, '%s\n', linea);
            linea = fgetl(fid_in_cpp);
        end  
    end
end

disp("Real Time Plugin generated");

fclose(fid_out_cpp);
fclose(fid_out_h);
clear;
