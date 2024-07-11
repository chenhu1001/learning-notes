# 钩子函数
```
<template>
  <div>
    <p>{{ message }}</p>
  </div>
</template>

<script>
export default {
  data() {
    return {
      message: 'Hello, Vue.js!',
    };
  },
  // 在实例被创建之前调用，此时组件的数据观测和事件配置尚未初始化
  beforeCreate() {
    console.log('beforeCreate');
  },
  // 实例已经创建完成后调用，此时组件的数据观测和事件配置已完成，但 DOM 尚未生成，无法访问 DOM
  created() {
    console.log('created');
  },
  // 在挂载开始之前被调用，此时模板编译已完成，但尚未将模板渲染到页面中
  beforeMount() {
    console.log('beforeMount');
  },
  // 在挂载完成后被调用，此时组件已经被渲染到页面中，可以访问到 DOM 元素
  mounted() {
    console.log('mounted');
  },
  // 在数据更新之前被调用，发生在虚拟 DOM 重新渲染和打补丁之前
  beforeUpdate() {
    console.log('beforeUpdate');
  },
  // 在数据更新之后被调用，发生在虚拟 DOM 重新渲染和打补丁之后
  updated() {
    console.log('updated');
  },
  // 在实例销毁之前调用，此时组件仍然完全可用
  beforeDestroy() {
    console.log('beforeDestroy');
  },
  // 在实例销毁之后调用，此时组件已被销毁，清理工作已完成
  destroyed() {
    console.log('destroyed');
  },
};
</script>

<style scoped>
p {
  font-size: 18px;
  color: blue;
}
</style>
```
# 数据选项
```
<template>
  <div>
    <p>Data: {{ dataValue }}</p>
    <p>Computed: {{ computedValue }}</p>
    <p>Props: {{ propValue }}</p>
    <button @click="increment">Increment</button>
  </div>
</template>

<script>
export default {
  props: {
    propValue: {
      type: String,
      default: 'Default Prop Value',
    },
  },
  data() {
    return {
      dataValue: 0,
    };
  },
  propsData: {
    propValue: 'Props Data Value',
  },
  computed: {
    computedValue() {
      return this.dataValue * 2;
    },
  },
  methods: {
    increment() {
      this.dataValue++;
    },
  },
  watch: {
    dataValue(newValue) {
      console.log('Data Value changed:', newValue);
    },
  },
};
</script>

<style scoped>
p {
  font-size: 18px;
  color: blue;
}
</style>
```
# 过滤器
```
<!DOCTYPE html>
<html lang="en">

<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Filter with Parameters Example</title>
</head>

<body>
  <div id="app">
    <p>{{ message | truncate(10) }}</p>
  </div>

  <script src="https://cdn.jsdelivr.net/npm/vue/dist/vue.js"></script>
  <script>
    Vue.filter('truncate', function (value, length) {
      if (value.length <= length) {
        return value;
      } else {
        return value.substring(0, length) + '...';
      }
    });

    new Vue({
      el: '#app',
      data() {
        return {
          message: 'This is a long message that needs to be truncated.',
        };
      },
    });
  </script>
</body>

</html>
```
