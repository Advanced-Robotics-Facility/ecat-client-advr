function blkStruct = slblocks

blkStruct.OpenFcn = {'XBotBlockToolbox'};
Browser(1).Library = 'XBotBlockToolbox';
Browser(1).Name    = 'XBotBlock Toolbox';

%%Browser.IsFlat  =  0;

blkStruct.Browser =  Browser;
