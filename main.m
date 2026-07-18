%% Projeto Final - ELT 537
%% 2026-07-17

clc
clear
close all

%% Rotina para buscar pasta raiz
FolderCurrent = which(mfilename);
FolderKey = '/AuRoRA_beta';
FolderRootId = strfind(FolderCurrent,FolderKey);
FolderRoot = FolderCurrent(1:FolderRootId(end)+numel(FolderKey)-1);
addpath(genpath(FolderRoot))

sim = Simulacao();
sim.simular();