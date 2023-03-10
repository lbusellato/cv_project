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
params.set = 1; % Image set to use
% RANSAC
params.ransac = true; % Use RANSAC or not
params.ransac_thresh = 0.01; % Inlier tolerance for RANSAC on features
params.ransac_iter = 1000; % Maximum iterations for RANSAC
% HOMOGRAPHY
params.pixel_tolerance = 5; % Inlier tolerance for RANSAC on homography
% IMAGE BLENDING
params.blending = 'linear'; % 'none', 'average', 'linear'

%% MOSAICING

mosaic = mosaicing(params);
figure(); title('Resulting mosaic'); hold on; imshow(mosaic);