### Docker 技术点详解：

1. **Namespace（命名空间）**：
   - Docker 使用命名空间来隔离容器之间的资源。常见的命名空间有：
     - **PID Namespace**：用于隔离进程ID。
     - **NET Namespace**：用于隔离网络接口。
     - **IPC Namespace**：用于隔离进程间通信。
     - **MNT Namespace**：用于隔离文件系统挂载。
     - **UTS Namespace**：用于隔离主机名和域名。
     - **USER Namespace**：用于隔离用户和用户组ID。
     - **CGROUP Namespace**：用于隔离容器资源的限制和监控。

2. **Cgroup（控制组）**：
   - 用于限制、计量、监控和隔离容器使用的资源，如 CPU、内存、磁盘 I/O 等。Docker 使用 Cgroup 来实现对容器资源的限制，例如：
     - CPU 限制
     - 内存限制
     - IO 限制
     - 网络带宽限制

3. **Rootfs（根文件系统）**：
   - 每个 Docker 容器都有自己的文件系统。根文件系统是容器的基础文件系统，可以通过 Docker 镜像提供。Docker 容器通过使用共享的基础镜像来提高效率。

4. **容器网络**：
   - **Bridge**：默认的网络驱动，Docker 会为每个容器分配一个独立的 IP 地址，并且容器之间的通信通过虚拟桥接网络进行。
   - **Host**：容器共享宿主机的网络栈，没有隔离，容器和宿主机共享 IP 地址。

### Kubernetes（K8s）架构详解：

1. **Master 节点**：
   - 集群的控制和管理节点，负责整个 Kubernetes 集群的管理和调度。包含以下组件：
     - **APIServer**：Kubernetes 集群的入口点，提供 RESTful API 供用户操作和管理集群。
     - **Kube-controller-manager**：负责控制器的管理，确保集群的期望状态与实际状态一致。
     - **Kube-scheduler**：负责根据集群资源和调度策略，将 Pod 安排到合适的 Node 上运行。
   
2. **Node 节点**：
   - 承载实际工作负载的节点，包含运行容器的计算资源。每个 Node 节点上都包含：
     - **Kubelet**：一个在每个节点上运行的代理，确保容器按照规范运行。
     - **Kube-proxy**：管理节点的网络规则，确保集群内部的服务发现和负载均衡。

3. **etcd**：
   - 一个分布式的键值存储系统，保存 Kubernetes 集群的所有配置信息和状态数据，包括 Pod、Service、ConfigMap、Secret 等。

4. **K8s 资源对象**：
   - **Pod**：Kubernetes 中最小的可部署单元，通常包含一个或多个容器。
   - **ReplicaSet**：确保在任何时候都有指定数量的 Pod 副本在运行，保证应用的高可用性。
   - **Deployment**：用于管理 ReplicaSet，可以轻松地进行应用部署、滚动更新等。
   - **DaemonSet**：确保每个 Node 上运行一个 Pod 副本，常用于集群范围的服务，如日志收集器。
   - **Service**：定义了一组 Pod 的访问规则，并提供负载均衡。常见类型包括 ClusterIP、NodePort 和 LoadBalancer。
   - **Ingress**：管理外部访问服务的规则，通常是 HTTP 或 HTTPS 访问路径的配置。
   - **StatefulSet**：为有状态的应用提供管理，保证 Pod 的顺序部署和稳定的网络身份。
   - **Job & CronJob**：Job 用于一次性任务的管理，CronJob 用于定时任务的管理。
   - **ConfigMap & Secret**：分别用于存储配置信息和敏感数据（如数据库密码）。

5. **资源限制**：
   - Kubernetes 使用 cgroup 来限制容器的 CPU、内存等资源，保证容器不会过度占用宿主机资源。

6. **健康检查**：
   - **LivenessProbe**：用于检测容器是否处于正常运行状态。如果探测失败，容器会被重启。
   - **ReadinessProbe**：用于检测容器是否准备好接受流量。如果探测失败，服务不被路由到该容器。

7. **容器生命周期钩子**：
   - 用于定义容器在生命周期中的特定事件，例如容器启动前或停止时执行的操作。

8. **Kubernetes 网络**：
   - Kubernetes 提供了多种网络插件来确保 Pod 间的网络通信，通常使用 Flannel、Calico 等网络插件。

9. **Kubernetes 客户端 - kubectl**：
   - `kubectl` 是 Kubernetes 的命令行工具，用户通过它与 Kubernetes 集群进行交互，管理和操作集群中的资源。

### 总结：Docker 和 Kubernetes 架构对比

- **Docker** 提供了一个单机的容器化解决方案，它通过 namespace 和 cgroup 隔离和限制容器资源，并通过网络驱动（如 bridge 和 host）提供容器的网络支持。每个 Docker 容器有独立的文件系统（rootfs）。
  
- **Kubernetes** 是一个容器编排平台，用于管理多台机器上的容器。它通过 Master 节点的 API 组件进行调度、控制、管理，而 Node 节点承载实际的工作负载。Kubernetes 使用 Pod 作为最小部署单元，支持高可用的服务发现、负载均衡和自动化运维。

在实际应用中，Docker 和 Kubernetes 是相辅相成的，Docker 用于容器化应用，而 Kubernetes 则用于大规模管理这些容器。
