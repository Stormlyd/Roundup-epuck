#ifndef STYLE_H
#define STYLE_H
#include <QString>

//边框
const QString border_info_1 = "border:1px solid #68d8fe;";
const QString border_info_2 = "border:2px solid #68d8fe;";
//const QString border_info_3 = "border:2px solid #00BFFF;";
const QString border_info_3 = "border:2px solid #fffFFF;";

const QString border_danger_1 = "border:1px solid #ff0000;";
const QString border_danger_2 = "border:2px solid #ff0000;";

//圆角
const QString rounded_sm = "border-radius: 4px;";
const QString rounded_md = "border-radius: 8px;";
const QString rounded_lg = "border-radius: 12px;";

//边距
const QString m_0  = "margin:0px;";
const QString m_4  = "margin:4px;";
const QString mt_4  = "margin-top:4px;";
const QString mb_4  = "margin-bottom:4px;";
const QString ml_4  = "margin-left:4px;";
const QString mr_4  = "margin-right:4px;";
const QString m_12 = "margin:12px;";
const QString mt_12  = "margin-top:12px;";
const QString mb_12  = "margin-bottom:12px;";
const QString ml_12  = "margin-left:12px;";
const QString mr_12  = "margin-right:12px;";
const QString p_4 = "padding:4px;";
const QString pt_4  = "padding-top:4px;";
const QString pb_4  = "padding-bottom:4px;";
const QString pl_4  = "padding-left:4px;";
const QString pr_4  = "padding-right:4px;";
const QString p_8 = "padding:8px;";
const QString pt_8  = "padding-top:8px;";
const QString pb_8  = "padding-bottom:8px;";
const QString pl_8  = "padding-left:8px;";
const QString pr_8  = "padding-right:8px;";
const QString p_12 = "padding:8px;";
const QString pt_12  = "padding-top:12px;";
const QString pb_12  = "padding-bottom:12px;";
const QString pl_12  = "padding-left:12px;";
const QString pr_12  = "padding-right:12px;";

//字体
const QString font_size_sm = "font-size:16px;";
const QString font_size_md = "font-size:20px;";
const QString font_size_lg = "font-size:24px;";
const QString font_size_lg1 = "font-size:40px;";
const QString font_size_28 = "font-size:28px;";

const QString font_weight_100 = "font-weight:100;";
const QString font_weight_200 = "font-weight:200;";
const QString font_weight_300 = "font-weight:300;";
const QString font_weight_400 = "font-weight:400;";
const QString font_weight_500 = "font-weight:500;";
const QString font_weight_600 = "font-weight:600;";
const QString font_weight_700 = "font-weight:700;";
const QString font_weight_900 = "font-weight:900;";

const QString text_center = "text-align:center;";
const QString text_left = "text-align:left;";
const QString text_right = "text-align:right;";

//颜色
const QString text_info = "color:#68d8fe;";
const QString text_white = "color:#ffffff;";
const QString text_danger = "color:#ff0000;";
//const QString text_cover = "color:#00BFFF;";
const QString text_cover = "color:#fffFFF;";
const QString text_select = "color:#edcb6a;";
//背景
const QString transparent_bg = "background:transparent;";
const QString info_bg = "background-color:rgb(255,255,255,25);";

const QString bg_info = "background-color:#68d8fe;";
const QString bg_info1 = "background-color:#101129;";

const QString mainButtonSelect = "QPushButton{"
                                 "border:2px solid #00c0ff;" //边框
                                 "border-radius: 12px;" //圆角
                                 "background-color:#00c0ff;"
                                 "margin:12px;"                     //边距
                                 "font-size:24px;"                   //字体
                                 "font-weight:500;"
                                 "color:#fffFFF;"                              //字体颜色
                                 "}";

//add by chenlu
//主界面按钮样式
const QString mainbuttonstyle="QPushButton{"
                              "border:2px solid #68d8fe;" //边框
                              "border-radius: 12px;" //圆角
                              "margin:12px;"                     //边距
                              "font-size:24px;"                   //字体
                              "font-weight:500;"
                              "color:#68d8fe;"                       //字体颜色
                              "}"
                              //鼠标悬停样式
                              "QPushButton:hover{"
                              "border:2px solid #00c0ff;" //边框
                              "border-radius: 12px;" //圆角
                              "background-color:#00c0ff;"
                              "margin:12px;"                     //边距
                              "font-size:24px;"                   //字体
                              "font-weight:500;"
                              "color:#fffFFF;"                       //字体颜色
                               "}"
                              //鼠标按下样式
                              "QPushButton:pressed{"
                              "border:2px solid #00c0ff;" //边框
                              "border-radius: 12px;" //圆角
                              "background-color:#00c0ff;"
                              "margin:12px;"                     //边距
                              "font-size:24px;"                   //字体
                              "font-weight:500;"
                              "color:#fffFFF;"                       //字体颜色
                               "}";



//主界面后台管理按钮样式
const QString adminBtnstyle="QPushButton{"
                            "border:2px solid #68d8fe;" //边框
                            "color:#68d8fe;"                       //字体颜色
                            "font-size:16px;" //字体
                            "font-weight:500;"
                            "border-radius: 20px;"
                            "}"
                            //鼠标悬停样式
                            "QPushButton:hover{"
                            "border:2px solid #fffFFF;" //边框
                            "color:#fffFFF;"                       //字体颜色
                            "font-size:16px;" //字体
                            "font-weight:500;"
                            "border-radius: 20px;"
                             "}"
                            //鼠标按下样式
                            "QPushButton:pressed{"
                            "border:2px solid #68d8fe;" //边框
                            "color:#68d8fe;"                       //字体颜色
                            "font-size:16px;" //字体
                            "font-weight:500;"
                            "border-radius: 20px;"
                             "}";

const QString closeBtnstyle="QPushButton{"
                            "border:2px solid #68d8fe;" //边框
                            "color:#68d8fe;"                       //字体颜色
                            "font-size:16px;" //字体
                            "font-weight:500;"
                            "border-radius: 20px;"
                            "}"
                            //鼠标悬停样式
                            "QPushButton:hover{"
                            "border:2px solid #68d8fe;" //边框
                            "color:#68d8fe;"                       //字体颜色
                            "font-size:16px;" //字体
                            "font-weight:500;"
                            "border-radius: 20px;"
                             "}"
                            //鼠标按下样式
                            "QPushButton:pressed{"
                            "border:2px solid #68d8fe;" //边框
                            "color:#68d8fe;"                       //字体颜色
                            "font-size:16px;" //字体
                            "font-weight:500;"
                            "border-radius: 20px;"
                             "}";


//后台界面按钮样式
const QString adminPagebtnstyle="QPushButton{"
                                "border:2px solid #68d8fe;" //边框
                                "color:#68d8fe;"                       //字体颜色
                                "border-radius: 8px;" //圆角
                                "font-size:20px;"                    //字体
                                "font-weight:600;"
                                "}"
                                //鼠标悬停样式
                                "QPushButton:hover{"
                                "border:2px solid #fffFFF;" //边框
                                "color:#fffFFF;"                       //字体颜色
                                "border-radius: 8px;" //圆角
                                "font-size:20px;"                    //字体
                                "font-weight:600;"
                                 "}"
                                //鼠标按下样式
                                "QPushButton:pressed{"
                                "border:2px solid #68d8fe;" //边框
                                "color:#68d8fe;"                       //字体颜色
                                "border-radius: 8px;" //圆角
                                "font-size:20px;"                    //字体
                                "font-weight:600;"
                                 "}";


//回到前台
const QString closeBtnStyle="QPushButton{"
                            "border:2px solid #68d8fe;" //边框
                            "color:#68d8fe;"                       //字体颜色
                            "border-radius: 20px;" //圆角
                            "font-size:16px;"                    //字体
                            "font-weight:500;"
                            "}"
                            //鼠标悬停样式
                            "QPushButton:hover{"
                            "border:2px solid #fffFFF;" //边框
                            "color:#fffFFF;"                       //字体颜色
                            "border-radius: 20px;" //圆角
                            "font-size:16px;"                    //字体
                            "font-weight:500;"
                             "}"
                            //鼠标按下样式
                            "QPushButton:pressed{"
                            "border:2px solid #68d8fe;" //边框
                            "color:#68d8fe;"                       //字体颜色
                            "border-radius: 20px;" //圆角
                            "font-size:16px;"                    //字体
                            "font-weight:500;"
                             "}";



//选择构成机器人数量按钮
const QString selectBtnStyle="QPushButton{"
                             "background:transparent;"
                             "color:#68d8fe;"                       //字体颜色
                             "font-weight:600;"
                             "font-size:16px;"                    //字体
                             "text-align:left;"
                             "}"
                             //鼠标悬停样式
                             "QPushButton:hover{"
                             "background:transparent;"
                             "color:#fffFFF;"                       //字体颜色
                             "font-weight:600;"
                             "font-size:16px;"                    //字体
                             "text-align:left;"
                              "}"
                             //鼠标按下样式
                             "QPushButton:pressed{"
                             "background:transparent;"
                             "color:#68d8fe;"                       //字体颜色
                             "font-weight:600;"
                             "font-size:16px;"                    //字体
                             "text-align:left;"
                              "}";


//保存按钮
const QString saveFigureBtnStyle="QPushButton{"
                                 "border:2px solid #68d8fe;" //边框
                                 "color:#68d8fe;"                       //字体颜色
                                 "border-radius: 4px;" //圆角
                                 "font-size:16px;"                    //字体
                                 "font-weight:600;"
                                 "}"
                                 //鼠标悬停样式
                                 "QPushButton:hover{"
                                 "border:2px solid #fffFFF;" //边框
                                 "color:#fffFFF;"                       //字体颜色
                                 "border-radius: 4px;" //圆角
                                 "font-size:16px;"                    //字体
                                 "font-weight:600;"
                                  "}"
                                 //鼠标按下样式
                                 "QPushButton:pressed{"
                                 "border:2px solid #68d8fe;" //边框
                                 "color:#68d8fe;"                       //字体颜色
                                 "border-radius: 4px;" //圆角
                                 "font-size:16px;"                    //字体
                                 "font-weight:600;"
                                  "}";

const QString savePassBtnStyle="QPushButton{"
                               "border:1px solid #68d8fe;" //边框
                               //"background-color:#68d8fe;"
                               "color:#68d8fe;"                      //字体颜色
                               "border-radius: 4px;" //圆角
                               "font-size:16px;"                    //字体
                               "font-weight:500;"
                               "}"
                               //鼠标悬停样式
                               "QPushButton:hover{"
                               "border:1px solid #68d8fe;" //边框
                               "background-color:#68d8fe;"
                               "color:#ffffff;"                      //字体颜色
                               "border-radius: 4px;" //圆角
                               "font-size:16px;"                    //字体
                               "font-weight:500;"
                                "}"
                               //鼠标按下样式
                               "QPushButton:pressed{"
                               "border:1px solid #68d8fe;" //边框
                               "background-color:#68d8fe;"
                               "color:#ffffff;"                      //字体颜色
                               "border-radius: 4px;" //圆角
                               "font-size:16px;"                    //字体
                               "font-weight:500;"
                                "}";

const QString definiteBtnStyle="QPushButton{"
                               "border:2px solid #68d8fe;" //边框
                               "color:#68d8fe;"
                               "border-radius: 12px;"
                               "font-size:24px;"
                               "font-weight:600;"
                               "}"
                               //鼠标悬停样式
                               "QPushButton:hover{"
                               "border:2px solid #ffffff;" //边框
                               "color:#ffffff;"
                               "border-radius: 12px;"
                               "font-size:24px;"
                               "font-weight:600;"
                                "}"
                               //鼠标按下样式
                               "QPushButton:pressed{"
                               "border:2px solid #0000ff;" //边框
                               "color:#0000ff;"
                               "border-radius: 12px;"
                               "font-size:24px;"
                               "font-weight:600;"
                                "}";

const QString definiteBtnStyle1="QPushButton{"
                                "border:2px solid #68d8fe;" //边框
                                "color:#68d8fe;"
                                "border-radius: 12px;"
                                "font-size:24px;"
                                "font-weight:600;"
                                "}"
                                //鼠标悬停样式
                                "QPushButton:hover{"
                                "border:2px solid #ffffff;" //边框
                                "color:#ffffff;"
                                "border-radius: 12px;"
                                "font-size:24px;"
                                "font-weight:600;"
                                 "}"
                                //鼠标按下样式
                                "QPushButton:pressed{"
                                "border:2px solid #0000ff;" //边框
                                "color:#0000ff;"
                                "border-radius: 12px;"
                                "font-size:24px;"
                                "font-weight:600;"
                                 "}";


const QString definiteBtnStyle2="QPushButton{"
                                "border:2px solid #68d8fe;" //边框
                                "color:#68d8fe;"
                                "border-radius: 12px;"
                                "font-size:24px;"
                                "font-weight:600;"
                                "}"
                                //鼠标悬停样式
                                "QPushButton:hover{"
                                "border:2px solid #ffffff;" //边框
                                "color:#ffffff;"
                                "border-radius: 12px;"
                                "font-size:24px;"
                                "font-weight:600;"
                                 "}"
                                //鼠标按下样式
                                "QPushButton:pressed{"
                                "border:2px solid #0000ff;" //边框
                                "color:#0000ff;"
                                "border-radius: 12px;"
                                "font-size:24px;"
                                "font-weight:600;"
                                 "}";


#endif // STYLE_H
