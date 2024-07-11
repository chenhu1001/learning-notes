# python-windows
```
安装anaconda命令
sh Anaconda3-2022.10-Linux-x86_64.sh
一步一步确认安装

修改bashrc，并在最后一行添加以下PATH
vi ~/.bashrc
export PATH=$PATH:/home/anaconda3/bin
source ~/.bashrc

// 安装虚拟环境
conda create --name python385 python=3.8.5

// 激活虚拟环境
conda activate python385

// 删除虚拟环境
conda env remove --name python385

// 查看虚拟环境列表
conda info --envs

// 退出虚拟环境
conda deactivate

// 安装pytorch
conda install pytorch torchvision torchaudio cpuonly -c pytorch

// 后台运行jupyter
nohup jupyter notebook --allow-root --ip=0.0.0.0 --port=8888 &
```

