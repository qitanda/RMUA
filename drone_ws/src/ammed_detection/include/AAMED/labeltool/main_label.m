clc;clear;close all;

global push_key;

push_key = [];


% [filename, pathname] = uigetfile( ...
%     {'*.jpg;*.tif;*.png;*.gif','All Image Files';...
%     '*.*','All Files' },...
%     '��ѡ��Ҫ��ǵ�ͼƬ');

img = imread([filename, pathname]);




handle_fig = figure('name','Image Label Tools');
set(handle_fig,'windowkeypressfcn',@callback_keyboard_pression_fun);

imshow(img);
hold on;


while 1
    if strcmp(push_key, 'q') % ����q�˳����
        break;
    end
    pt = [];
    while 1
        if strcmp(push_key, 'return') % ���»س�������ǰ���
            break;
        end
    end
    
    pause(0.1);
end