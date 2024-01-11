%Computer Vision 2019/20
%Work 2 - Problem 3: Structure From Motion From Two Views
%Authors:
%Nº80998  Name: Pedro Miguel Menezes Ramalho
%Nº85183  Name: Ricardo Miguel Diogo de Oliveira Chin
%Nº93807  Name: Henrique Manuel Caldeira Pimentel Furtado

clc
clear all
close all
warning off

folder=strcat(pwd,'\Imagens\','\calibrationCylinder\','\calibrationSphere\');
addpath(folder)

fprintf('\nComputer Vision 2019/20\n');
fprintf('This program creates structure from motion from two views.\n');

selec = 0;
while(selec==0) 
    fprintf('\nPick an option:\n');
    fprintf('1: Sphere     2: Saved Plots Sphere 3D\n');
    fprintf('3: Cylinder   4: Saved Plots Cylinder 3D\n');
    fprintf('0: Exit program\n');
    fprintf('\n');
    option = input('');
    valid=0;
    
    switch(option)
        case 0
            valid=1;
        case 1
            close all
            run TestSphere
            valid=1;
        case 2
            run Plots3DSphere
            valid=1;          
        case 3
            close all
            run TestCylinder
            valid=1;            
        case 4
            run Plots3DCylinder
            valid=1;       
    end    
        if(valid==0)
            fprintf('\nError! Invalid choice!\n');
        end
       
        if valid == 1 && option ~= 0
            fprintf('Program loading...\n')
         
        elseif valid == 1 && option == 0
            selec=1; %Closes the loop
            fprintf('\nProgram ended.\n\n');       
        end  
        
end

clear folder option selec valid 