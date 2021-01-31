% Little helper script to add PARODIS to the path
% The script will add the current directory (pwd) to the path, recursively, but excluding the /examples/ directory
% Afterwards, it will test if YALMIP is installed/added to the path. If so, it will prompt users to perform yalmiptest

fprintf("Adding PARODIS directory to MATLAB Path...\n");
addpath(genpath(pwd));
rmpath(genpath([pwd filesep 'examples']));

fprintf("Saving path... ");
try
    savepath
    fprintf("Done\n");
catch exception
    fprintf("Could not save path: \n%s", getReport(exception));
end

if exist('yalmip')
    v = yalmip('version');
    fprintf("You have YALMIP version %s installed.\n", v);
    if str2double(v) < 20200000
        disp('You should consider updating to a newer release. <a href="https://yalmip.github.io/download/">https://yalmip.github.io/download/</a>');
    end
    
    choice = input('Would you like to run yalmiptest? This will test all available solvers. (y/N) ', 's');
    if strcmpi(choice, 'y')
        yalmiptest;
    end
    
    fprintf("Done. Have fun!\n");
else
    fprintf("No YALMIP installation found. PARODIS requires YALMIP to work. Have you added it to the path?\n");
    disp('<a href="https://yalmip.github.io/download/">https://yalmip.github.io/download/</a>')

end