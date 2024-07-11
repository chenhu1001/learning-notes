## 1、登录接口
**描述：** 

- 用户登录接口

**请求URL：** 
- ` http://xx.com/user/login `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
|username |是  |string |用户名   |
|password |是  |string | 密码    |

 **返回示例**

``` 
{
    "data": {
        "createTime": "2017-12-14 08:57:20",
        "createUser": "admin",
        "detailAddress": "成都市天府新区",
        "email": "admin@tianfugroup.com",
        "id": 1,
        "mobile": "2147483647",
        "modifyTime": "2017-12-21 08:57:32",
        "modifyUser": "admin",
        "nickName": "administrator",
        "passWord": "admin",
        "permissions": [
            {
                "children": [
                    {
                        "children": [],
                        "id": 2,
                        "isMenu": 0,
                        "isSystemPermission": 0,
                        "moduleId": 1,
                        "parentId": 1,
                        "permissionName": "新建用户",
                        "permissionType": 1,
                        "status": 1,
                        "url": "/userManager?method=addUser"
                    },
                    {
                        "children": [],
                        "id": 3,
                        "isMenu": 0,
                        "isSystemPermission": 0,
                        "moduleId": 1,
                        "parentId": 1,
                        "permissionName": "修改用户",
                        "permissionType": 1,
                        "status": 1,
                        "url": "/userManager?method=updateUser"
                    },
                    {
                        "children": [],
                        "id": 4,
                        "isMenu": 0,
                        "isSystemPermission": 0,
                        "moduleId": 1,
                        "parentId": 1,
                        "permissionName": "删除用户",
                        "permissionType": 1,
                        "status": 1,
                        "url": "/userManager?method=deleteUser"
                    }
                ],
                "id": 1,
                "isMenu": 1,
                "isSystemPermission": 1,
                "moduleId": 1,
                "parentId": 0,
                "permissionName": "用户管理",
                "permissionType": 1,
                "status": 1,
                "url": "#"
            },
            {
                "children": [
                    {
                        "children": [],
                        "id": 6,
                        "isMenu": 0,
                        "isSystemPermission": 0,
                        "moduleId": 2,
                        "parentId": 5,
                        "permissionName": "新建组织",
                        "permissionType": 1,
                        "status": 1,
                        "url": "/groupManager?method=addGroup"
                    },
                    {
                        "children": [],
                        "id": 7,
                        "isMenu": 0,
                        "isSystemPermission": 0,
                        "moduleId": 2,
                        "parentId": 5,
                        "permissionName": "修改组织",
                        "permissionType": 1,
                        "status": 1,
                        "url": "/groupManager?method=updateGroup"
                    }
                ],
                "id": 5,
                "isMenu": 1,
                "isSystemPermission": 1,
                "moduleId": 2,
                "parentId": 0,
                "permissionName": "组织管理",
                "permissionType": 1,
                "status": 1,
                "url": "#"
            }
        ],
        "qq": "567894321",
        "status": 1,
        "trueName": "administrator",
        "userName": "admin",
        "wechat": "admin003"
    },
    "expires": 3600,
    "msg": "操作成功",
    "statusCode": "0",
    "token": "eyJhbGciOiJIUzI1NiJ9.eyJpYXQiOjE1MTQxNjg4MDksInN1YiI6IntcInVzZXJuYW1lXCI6XCJhZG1pblwifSJ9.CTCSt7LHLbZoy_b0scsBN-llLjd3jDG8NpM_Y-Ig_Fs"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
|token |string   |token验证  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 2、模块列表 
**描述：** 

- 模块列表接口

**请求URL：** 
- ` http://xx.com/api/module/selectAll `
  
**请求方式：**
- POST 

**参数：** 
无

 **返回示例**

``` 
{
    "data": {
        "modules": [
            {
                "createTime": "2017-12-12 09:18:50",
                "createUser": "admin",
                "description": "管理用户的维护",
                "id": 1,
                "modifyTime": "2017-12-21 09:18:55",
                "modifyUser": "admin",
                "moduleName": "用户管理",
                "orderNum": 1
            },
            {
                "createTime": "2017-12-19 09:19:16",
                "createUser": "admin",
                "description": "管理组织的维护",
                "id": 2,
                "modifyTime": "2017-12-21 09:19:23",
                "modifyUser": "admin",
                "moduleName": "组织管理",
                "orderNum": 2
            },
            {
                "createTime": "2017-12-20 09:19:49",
                "createUser": "admin",
                "description": "管理角色的维护",
                "id": 3,
                "modifyTime": "2017-12-21 09:19:56",
                "modifyUser": "admin",
                "moduleName": "角色管理",
                "orderNum": 3
            },
            {
                "createTime": "2017-12-13 09:20:21",
                "createUser": "admin",
                "description": "管理模块的维护",
                "id": 4,
                "modifyTime": "2017-12-21 09:20:29",
                "modifyUser": "admin",
                "moduleName": "模块管理",
                "orderNum": 4
            },
            {
                "createTime": "2017-12-15 09:20:51",
                "createUser": "admin",
                "description": "管理权限的维护",
                "id": 5,
                "modifyTime": "2017-12-21 09:20:59",
                "modifyUser": "admin",
                "moduleName": "权限管理",
                "orderNum": 5
            },
            {
                "createTime": "2017-12-16 09:21:16",
                "createUser": "admin",
                "description": "管理系统的维护",
                "id": 6,
                "modifyTime": "2017-12-21 09:21:23",
                "modifyUser": "admin",
                "moduleName": "系统管理",
                "orderNum": 6
            },
            {
                "createTime": "2017-12-26 11:22:40",
                "id": 7,
                "modifyTime": "2017-12-26 11:24:16",
                "moduleName": "用户管理1"
            }
        ]
    },
    "expires": 0,
    "msg": "操作成功",
    "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
|moduleName |string   |模块名  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 3、菜单列表
**描述：** 

- 菜单列表接口

**请求URL：** 
- ` http://xx.com/module/selectMenu `
  
**请求方式：**
- POST 

**参数：** 
无

 **返回示例**

``` 
{
    "data": {
        "menus": [
            {
                "id": 1,
                "moduleName": "用户管理"
            },
            {
                "id": 2,
                "moduleName": "组织管理"
            },
            {
                "id": 3,
                "moduleName": "角色管理"
            },
            {
                "id": 4,
                "moduleName": "模块管理"
            },
            {
                "id": 5,
                "moduleName": "权限管理"
            },
            {
                "id": 6,
                "moduleName": "系统管理"
            },
            {
                "id": 7,
                "moduleName": "用户管理1"
            },
            {
                "id": 10,
                "moduleName": "用户管理4"
            }
        ]
    },
    "expires": 0,
    "msg": "操作成功",
    "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| moduleName |string   |模块名  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 4、新增模块
**描述：** 

- 新增模块接口

**请求URL：** 
- ` http://xx.com/module/save `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
|moduleName |是  |string |模块名   |

 **返回示例**

``` 
{
    "expires": 0,
    "msg": "操作成功",
    "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 5、修改模块
**描述：** 

- 修改模块接口

**请求URL：** 
- ` http://xx.com/module/update `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
|id |是  |int |模块id   |
|moduleName |是  |string |模块名   |

 **返回示例**

``` 
{
    "expires": 0,
    "msg": "操作成功",
    "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 6、删除模块
**描述：** 

- 删除模块接口

**请求URL：** 
- ` http://xx.com/module/delete `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| moduleIds |是  |string |模块id集合-多个用竖线\|隔开   |

 **返回示例**

``` 
{
    "expires": 0,
    "msg": "操作成功",
    "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 7、用户列表
**描述：** 

- 用户列表接口

**请求URL：** 
- ` http://xx.com/user/getUserInfoList `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| pageSize |是  |int |分页大小   |
| currentPage |是  |int |当前分页   |
| userName |否  |string |用户名   |
| trueName |否  |string |真实姓名   |
| roles |否  |string |多个角色名   |
| all |否  |string |筛选条件-按userName-trueName-roles   |

 **返回示例**

``` 
{
  "data": {
    "currentPage": 1,
    "datas": [
      {
        "createTime": "2017-12-14 08:57:20",
        "id": 1,
        "roles": "总监,开发人员,总经理",
        "status": 1,
        "trueName": "administrator",
        "userName": "admin"
      }
    ],
    "pageSize": 0,
    "totals": 1
  },
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| roles |string   |多个角色名  |
| trueName |string   |真实姓名  |
| userName |string   |用户名  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 8、新增用户
**描述：** 

- 新增用户接口

**请求URL：** 
- ` http://xx.com/user/addUser `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| userName |是  |string |用户名   |
| password |是  |string |密码   |
| trueName |否  |string |真实姓名   |
| nickName |否  |string |昵称   |
| mobile |否  |string |手机号   |
| detailAddress |否  |string |详细地址   |
| email |否  |string |电子邮箱   |
| qq |否  |string |QQ   |
| wechat |否  |string |微信   |
| status |否  |int |状态   |
| groupids |否  |string |组织id组   |
| roleids |否  |string |角色id组   |

 **返回示例**

``` 
{
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 9、角色列表
**描述：** 

- 角色列表接口

**请求URL：** 
- ` http://xx.com/role/getRoleList `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| pageSize |是  |int |分页大小   |
| currentPage |是  |int |当前分页   |
| selectAll |否  |string |全部   |
| roleName |否  |string |角色名   |
| createUser |否  |string |创建用户   |

 **返回示例**

``` 
{
  "data": {
    "currentPage": 3,
    "datas": [
      {
        "createTime": 1513559114000,
        "createUser": "admin",
        "description": "负责全局事务",
        "id": 1,
        "isSystemRole": 1,
        "modifyTime": 1513645504000,
        "modifyUser": "admin",
        "roleName": "总经理",
        "status": 1
      },
      {
        "createTime": 1513645552000,
        "createUser": "admin",
        "description": "协助总经理",
        "id": 2,
        "isSystemRole": 0,
        "modifyTime": 1513731945000,
        "modifyUser": "admin",
        "roleName": "总监",
        "status": 1
      },
      {
        "createTime": 1513731985000,
        "createUser": "admin",
        "description": "实现具体功能",
        "id": 3,
        "isSystemRole": 1,
        "modifyTime": 1513818374000,
        "modifyUser": "admin",
        "roleName": "开发人员",
        "status": 1
      }
    ],
    "pageSize": 2,
    "totals": 1
  },
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 10、新增角色
**描述：** 

- 新增角色接口

**请求URL：** 
- ` http://xx.com/role/addRole `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| roleName |是  |string |角色名   |
| createTime |否  |string |创建时间   |
| createUser |否  |string |创建用户   |
| description |是  |string |描述   |
| isSystemRole |否  |int |是否系统角色   |
| modifyTime |否  |string |修改时间   |
| modifyUser |否  |string |修改用户   |
| status |否  |int |状态   |

 **返回示例**

``` 
{
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 11、修改角色
**描述：** 

- 修改角色接口

**请求URL：** 
- ` http://xx.com/role/update `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| id |是  |int |角色id   |
| roleName |否  |string |角色名   |
| createTime |否  |string |创建时间   |
| createUser |否  |string |创建用户   |
| description |否  |string |描述   |
| isSystemRole |否  |int |是否系统角色   |
| modifyTime |否  |string |修改时间   |
| modifyUser |否  |string |修改用户   |
| status |否  |int |状态   |

 **返回示例**

``` 
{
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 12、删除角色
**描述：** 

- 删除角色接口

**请求URL：** 
- ` http://xx.com/role/deleteRoles `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| roleIds |是  |string |角色ids   |

 **返回示例**

``` 
{
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 13、查询角色
**描述：** 

- 查询角色接口

**请求URL：** 
- ` http://xx.com/role/getRoleInfoById `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| id |是  |int |角色id   |

 **返回示例**

``` 
{
  "data": {
    "createTime": 1513731985000,
    "createUser": "admin",
    "description": "实现具体功能",
    "id": 3,
    "isSystemRole": 1,
    "modifyTime": 1513818374000,
    "modifyUser": "admin",
    "roleName": "开发人员",
    "status": 1
  },
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 14、角色用户列表
**描述：** 

- 角色用户列表接口

**请求URL：** 
- ` http://xx.com/userRole/getRoleUserList `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| id |是  |int |角色id   |

 **返回示例**

``` 
{
  "data": [
    {
      "createTime": "2017-12-14 08:57:20",
      "createUser": "admin",
      "detailAddress": "成都市天府新区",
      "email": "admin@tianfugroup.com",
      "id": 1,
      "mobile": "2147483647",
      "modifyTime": "2017-12-21 08:57:32",
      "modifyUser": "admin",
      "nickName": "administrator",
      "passWord": "admin",
      "qq": "567894321",
      "status": 1,
      "trueName": "administrator",
      "userName": "admin",
      "wechat": "admin003"
    },
    {
      "createTime": "2017-12-27 09:18:53",
      "createUser": "admin",
      "detailAddress": "成都市青羊区",
      "email": "jie.liu@tianfugroup.com",
      "id": 170,
      "mobile": "13666668899",
      "modifyTime": "2017-12-27 09:18:45",
      "modifyUser": "admin",
      "nickName": "jackYao",
      "passWord": "123456",
      "qq": "277139277",
      "status": 2,
      "trueName": "刘杰",
      "userName": "liuj",
      "wechat": "yao277"
    }
  ],
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 15、删除角色用户
**描述：** 

- 删除角色用户接口

**请求URL：** 
- ` http://xx.com/userRole/deleteRoleUser `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| roleId |是  |int |角色id   |
| userIds |是  |string |用户ids   |

 **返回示例**

``` 
{
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 16、添加角色用户
**描述：** 

- 添加角色用户接口

**请求URL：** 
- ` http://xx.com/userRole/addRoleUser `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| roleId |是  |int |角色id   |
| userIds |是  |string |用户ids   |

 **返回示例**

``` 
{
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 17、清空角色用户
**描述：** 

- 清空角色用户接口

**请求URL：** 
- ` http://xx.com/userRole/emptyRoleUser `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| roleId |是  |int |角色id   |

 **返回示例**

``` 
{
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 18、组织树
**描述：** 

- 组织树接口

**请求URL：** 
- ` http://xx.com/organization/organizationTree `
  
**请求方式：**
- POST 

**参数：** 
无

 **返回示例**

``` 
{
  "data": {
    "datas": [
      {
        "children": [
          {
            "children": [
              {
                "children": [],
                "id": 5,
                "label": "法律合规部",
                "pid": 2
              },
              {
                "children": [],
                "id": 6,
                "label": "互联网金融事业部",
                "pid": 2
              }
            ],
            "id": 2,
            "label": "天府银行",
            "pid": 1
          },
          {
            "children": [
              {
                "children": [],
                "id": 8,
                "label": "科技信息事业部",
                "pid": 3
              },
              {
                "children": [],
                "id": 9,
                "label": "矿业能源事业部",
                "pid": 3
              }
            ],
            "id": 3,
            "label": "天府集团",
            "pid": 1
          },
          {
            "children": [
              {
                "children": [],
                "id": 10,
                "label": "风险合规部",
                "pid": 4
              },
              {
                "children": [],
                "id": 11,
                "label": "业务发展部",
                "pid": 4
              }
            ],
            "id": 4,
            "label": "天府金租",
            "pid": 1
          }
        ],
        "id": 1,
        "label": "三位一体",
        "pid": 0
      }
    ],
    "nodeDefaultCheckBox": [
      1
    ]
  },
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 19、更新用户状态
**描述：** 

- 更新用户状态接口

**请求URL：** 
- ` http://xx.com/user/updateUserStatus `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| userIds |是  |string |用户ids   |
| status |是  |int |状态-0为删除-1为启用-2为停用   |

 **返回示例**

``` 
{
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 20、更新密码
**描述：** 

- 清空角色用户接口

**请求URL：** 
- ` http://xx.com/user/updatePassword `
  
**请求方式：**
- POST 

**参数：** 

管理员修改密码  

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| modifyPwdType |是  |int |固定为0   |
| passWord |是  |string |密码   |
| id |是  |int |用户id   |

用户自己修改密码

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| modifyPwdType |是  |int |固定为1   |
| passWord |是  |string |原密码   |
| id |是  |int |用户id   |
| newPassword |是  |string |新密码   |

 **返回示例**

``` 
{
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 21、修改用户
**描述：** 

- 修改用户接口

**请求URL：** 
- ` http://xx.com/user/updateUserInfo `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| id |是  |int |用户id   |
| userName |否  |string |用户名   |
| trueName |否  |string |真实姓名   |
| nickName |否  |string |昵称   |
| mobile |否  |string |手机号   |
| detailAddress |否  |string |详细地址   |
| email |否  |string |电子邮箱   |
| qq |否  |string |QQ   |
| wechat |否  |string |微信   |
| status |否  |int |状态   |
| groupids |否  |string |组织id组   |
| roleids |否  |string |角色id组   |

 **返回示例**

``` 
{
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 22、查询用户组织角色信息
**描述：** 

- 查询用户组织角色信息接口

**请求URL：** 
- ` http://xx.com/user/getUserInfoByUserId `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| id |是  |int |用户id   |

 **返回示例**

``` 
{
  "data": {
    "checkedRoleids": [
      {
        "id": 1,
        "roleName": "总经理1"
      },
      {
        "id": 3,
        "roleName": "开发人员1"
      }
    ],
    "roleids": [
      {
        "createTime": 1513645552000,
        "createUser": "chenhu1",
        "description": "协助总经理",
        "id": 2,
        "isSystemRole": 0,
        "modifyTime": 1513731945000,
        "modifyUser": "admin",
        "roleName": "总监",
        "status": 1
      }
    ],
    "groupids": [
      {
        "id": 1,
        "organizationName": "科技信息事业部"
      },
      {
        "id": 2,
        "organizationName": "互联网金融事业部"
      }
    ]
  },
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 23、新增权限
**描述：** 

- 新增权限接口

**请求URL：** 
- ` http://xx.com/permission/savePermission `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| permissionName |是  |string |权限名   |
| parentId |是  |int |父id   |
| status |是  |int |状态   |
| permissionType |否  |int |权限类型   |
| isMenu |否  |int |是否是菜单   |
| url |否  |string |路径   |
| moduleId |否  |int |模块id   |
| isSystemPermission |否  |int |是否系统权限   |

 **返回示例**

``` 
{
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 24、更新权限
**描述：** 

- 更新权限接口

**请求URL：** 
- ` http://xx.com/permission/updatePermission `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| id |是  |int |权限id   |
| permissionName |是  |string |权限名   |
| parentId |是  |int |父id   |
| status |是  |int |状态   |
| permissionType |否  |int |权限类型   |
| isMenu |否  |int |是否是菜单   |
| url |否  |string |路径   |
| moduleId |否  |int |模块id   |
| isSystemPermission |否  |int |是否系统权限   |

 **返回示例**

``` 
{
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 25、删除权限
**描述：** 

- 删除权限接口

**请求URL：** 
- ` http://xx.com/permission/deletePermission `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| ids |是  |int |权限ids   |

 **返回示例**

``` 
{
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 26、查询权限
**描述：** 

- 查询全新接口

**请求URL：** 
- ` http://xx.com/permission/selectEntityOne `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| id |是  |int |权限id   |

 **返回示例**

``` 
{
  "data": {
    "createTime": "2017-12-20 09:24:01",
    "createUser": "admin",
    "id": 1,
    "isMenu": 1,
    "isSystemPermission": 1,
    "modifyTime": "2017-12-21 09:24:11",
    "modifyUser": "admin",
    "moduleId": 1,
    "parentId": 0,
    "permissionName": "用户管理",
    "permissionType": 1,
    "status": 1,
    "url": "#"
  },
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 27、权限树
**描述：** 

- 权限树接口

**请求URL：** 
- ` http://xx.com/permission/permissionTree `
  
**请求方式：**
- POST 

**参数：** 
无

 **返回示例**

``` 
{
  "data": [
    {
      "children": [
        {
          "children": [],
          "id": 2,
          "label": "新建用户"
        },
        {
          "children": [],
          "id": 3,
          "label": "修改用户"
        },
        {
          "children": [],
          "id": 4,
          "label": "删除用户1"
        }
      ],
      "id": 1,
      "label": "用户管理"
    },
    {
      "children": [
        {
          "children": [],
          "id": 6,
          "label": "新建组织"
        },
        {
          "children": [],
          "id": 7,
          "label": "修改组织"
        }
      ],
      "id": 5,
      "label": "组织管理"
    }
  ],
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 28、获取组织树子节点
**描述：** 

- 获取组织树子节点接口

**请求URL：** 
- ` http://xx.com/organization/getChildrenNode `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| id |是  |int |组织id   |

 **返回示例**

``` 
{
  "data": [
    {
      "abbreviation": "项目部",
      "createTime": "2017-12-20 09:16:15",
      "createUser": "admin",
      "description": "负责科技部项目相关",
      "id": 5,
      "modifyTime": "2017-12-21 09:16:21",
      "modifyUser": "admin",
      "orderNum": 2,
      "organizationName": "项目部",
      "parentId": 1,
      "status": 1
    },
    {
      "abbreviation": "技术部",
      "createTime": "2017-12-20 09:16:15",
      "createUser": "admin",
      "description": "负责科技部研发相关",
      "id": 6,
      "modifyTime": "2017-12-21 09:16:21",
      "modifyUser": "admin",
      "orderNum": 3,
      "organizationName": "技术部",
      "parentId": 1,
      "status": 1
    }
  ],
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 29、新增组织
**描述：** 

- 新增组织接口

**请求URL：** 
- ` http://xx.com/organization/addOrganization `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| id |是  |int |组织id   |
| parentId |是  |int |父id   |
| organizationName |是  |string |组织名   |
| description |否  |string |组织描述   |

 **返回示例**

``` 
{
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 30、删除组织
**描述：** 

- 删除组织接口

**请求URL：** 
- ` http://xx.com/organization/batchDeleteOrganization `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| ids |是  |string |组织ids   |

 **返回示例**

``` 
{
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 31、更新组织
**描述：** 

- 更新组织接口

**请求URL：** 
- ` http://xx.com/organization/updateOrganization `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| id |是  |int |组织id   |
| organizationName |否  |string |组织名   |
| description |否  |string |组织描述   |

 **返回示例**

``` 
{
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 32、查询组织用户信息
**描述：** 

- 查询组织用户信息接口

**请求URL：** 
- ` http://xx.com/organization/getUserInfoListByOrganizationId `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| id |是  |int |组织id   |
| pageSize |是  |int |分页大小   |
| currentPage |是  |int |当前页   |

 **返回示例**

``` 
{
  "data": {
    "currentPage": 1,
    "datas": [
      {
        "createTime": "2017-12-14 08:57:20",
        "createUser": "admin",
        "detailAddress": "成都市天府新区",
        "email": "admin@tianfugroup.com",
        "mobile": "2147483647",
        "nickName": "administrator",
        "qq": "567894321",
        "status": 1,
        "trueName": "administrator",
        "userName": "admin",
        "wechat": "admin003"
      },
      {
        "createTime": "2017-12-27 09:18:53",
        "createUser": "admin",
        "detailAddress": "成都市金牛区",
        "email": "xianlin.yao@tianfugroup.com",
        "mobile": "13666668899",
        "nickName": "jackYao",
        "qq": "277139277",
        "status": 2,
        "trueName": "姚先林",
        "userName": "yaoxianlin",
        "wechat": "yao277"
      },
      {
        "createTime": "2017-12-27 14:05:33",
        "createUser": "admin",
        "detailAddress": "成都市金牛区",
        "email": "xianlin.yao@tianfugroup.com",
        "mobile": "13666668899",
        "nickName": "jackYao",
        "qq": "277139277",
        "status": 0,
        "trueName": "刘杰",
        "userName": "lili",
        "wechat": "yao277"
      }
    ],
    "pageSize": 10,
    "totals": 3
  },
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 33、组织中新增用户
**描述：** 

- 组织中新增用户接口

**请求URL：** 
- ` http://xx.com/organization/addUserToOrganation `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| groupids |是  |int |组织id   |
| userIds |是  |string |用户ids   |

 **返回示例**

``` 
{
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 34、改变组织管理员
**描述：** 

- 改变组织管理员接口

**请求URL：** 
- ` http://xx.com/organization/changeUserToAdmin `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| groupids |是  |int | 组织id   |
| userIds |是  |string |用户ids   |
| isAdmin |是  |int |是否组织管理员   |

 **返回示例**

``` 
{
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 35、创建用户and添加组织
**描述：** 

- 创建用户and添加组织接口

**请求URL：** 
- ` http://xx.com/organization/createUserToOrganation `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |


|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| groupids |是  |int |组织id   |
| userName |是  |string |用户名   |
| password |是  |string |密码   |
| trueName |否  |string |真实姓名   |
| nickName |否  |string |昵称   |
| mobile |否  |string |手机号   |
| detailAddress |否  |string |详细地址   |
| email |否  |string |电子邮箱   |
| qq |否  |string |QQ   |
| wechat |否  |string |微信   |
| status |否  |int |状态   |
| groupids |否  |string |组织id组   |
| roleids |否  |string |角色id组   |

 **返回示例**

``` 
{
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 36、移动组织
**描述：** 

- 移动组织接口

**请求URL：** 
- ` http://xx.com/organization/moveOrganation `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| id |是  |int |组织id   |
| parentId |是  |int |父id   |

 **返回示例**

``` 
{
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 37、新增or更新角色权限
**描述：** 

- 新增or更新角色权限接口

**请求URL：** 
- ` http://xx.com/rolePermission/updateRolePermission `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| roleId |是  |int |角色id   |
| permissionIds |是  |string |权限ids  |

 **返回示例**

``` 
{
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 38、模块菜单
**描述：** 

- 模块菜单接口

**请求URL：** 
- ` http://xx.com/rolePermission/selectMenu `
  
**请求方式：**
- POST 

**参数：** 
无

 **返回示例**

``` 
{
  "data": {
    "menus": [
      {
        "id": 7,
        "moduleName": "用户管理1"
      },
      {
        "id": 10,
        "moduleName": "用户管理4"
      },
      {
        "id": 1,
        "moduleName": "用户管理"
      },
      {
        "id": 2,
        "moduleName": "组织管理"
      },
      {
        "id": 3,
        "moduleName": "角色管理"
      },
      {
        "id": 4,
        "moduleName": "模块管理"
      },
      {
        "id": 5,
        "moduleName": "权限管理"
      },
      {
        "id": 6,
        "moduleName": "系统管理"
      }
    ]
  },
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 39、权限树
**描述：** 

- 权限树接口

**请求URL：** 
- ` http://xx.com/organization/moveOrganation `
  
**请求方式：**
- POST 

**参数：** 

|参数名|必选|类型|说明|
|:----    |:---|:----- |-----   |
| roleId |是  |int |角色id   |
| moduleId |是  |int |模块id   |

 **返回示例**

``` 
{
  "data": {
    "checkedKeys": [
      "2",
      "3",
      "4"
    ],
    "checkedPermissionList": [
      {
        "children": [
          {
            "children": [],
            "id": 2,
            "label": "新建用户"
          },
          {
            "children": [],
            "id": 3,
            "label": "修改用户"
          },
          {
            "children": [],
            "id": 4,
            "label": "删除用户1"
          }
        ],
        "id": 1,
        "label": "用户管理"
      }
    ],
    "moduleId": 1,
    "roleId": 1
  },
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |
| msg |string   |状态信息  |

 **备注** 

- 更多返回错误代码请看错误代码描述

## 40、角色列表（不分页）
**描述：** 

- 角色列表（不分页）接口

**请求URL：** 
- ` http://xx.com/role/getRoleListWithoutPage `
  
**请求方式：**
- POST 

**参数：** 
无

 **返回示例**

``` 
{
  "data": [
    {
      "createTime": "2017-12-19 09:05:52",
      "createUser": "chenhu1",
      "description": "admin",
      "id": 2,
      "isSystemRole": 0,
      "modifyTime": "2017-12-20 09:05:45",
      "modifyUser": "admin",
      "roleName": "Administrator",
      "status": 1
    },
    {
      "createTime": "2017-12-20 09:06:25",
      "createUser": "admin",
      "description": "实现具体功能",
      "id": 3,
      "isSystemRole": 1,
      "modifyTime": "2017-12-21 09:06:14",
      "modifyUser": "admin",
      "roleName": "开发人员",
      "status": 1
    },
    {
      "description": "2",
      "id": 9,
      "roleName": "2",
      "status": 1
    },
    {
      "createTime": "2018-01-05 15:59:00",
      "createUser": "admin",
      "description": "3",
      "id": 10,
      "roleName": "3",
      "status": 1
    }
  ],
  "msg": "操作成功",
  "statusCode": "0"
}
```

 **返回参数说明** 

|参数名|类型|说明|
|:-----  |:-----|-----                           |
| statusCode |int   |状态码  |

 **备注** 

- 更多返回错误代码请看错误代码描述
