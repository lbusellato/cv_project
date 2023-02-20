%--------------------------------------------------------------------------
%
%                       AUTOMATIC 2D MOSAICING
%
%   Final project for the Computer Vision course of the Master's degree in
%   Computer Engineering for Robotics and Smart Industry @UniVR
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------

%% SETUP

rng(420); % To keep consistency between runs
warning('off', 'MATLAB:dispatcher:nameConflict');
clc; clearvars; close all force;
addpath(genpath("vlfeat-0.9.21/"));
addpath(genpath("images/"));
addpath(genpath("scripts/"));
addpath(genpath("mosaics/"));

%% PARAMETERS

% IMAGE SET
params.set = 1;
% RANSAC
params.ransac_thresh = 0.01;
params.ransac_iter = 200;
% HOMOGRAPHY
params.pixel_tolerance = 5;
% IMAGE BLENDING
params.alpha = 0.5;

%% MOSAICING

mosaic = mosaicing(params);
figure(); title('Resulting mosaic'); hold on; imshow(mosaic);