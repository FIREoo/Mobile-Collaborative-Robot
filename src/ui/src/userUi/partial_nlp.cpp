#include "mainwindow.h"
#include "string.h"
#include "ui_mainwindow.h"
#include <QAbstractItemView>
#include <QMainWindow>
#include <QStandardItemModel>
#include <QStringList>
#include <QStringListModel>
#include <QtWidgets>
#include <map>

#define C033 "\033[0;33m"
#define C094 "\033[0;94m"
#define C0 "\033[0m"

/*ros service*/
void MainWindow::on_pushButton_voice_cmd_clicked()
{
    ROS_INFO("\033[0;94m=====Start NLP=====\033[0m");
    ui->textEdit_voice->setText(QString::fromStdString(""));
    qApp->processEvents();
    //**service**//
    ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>("record_service");
    std_srvs::SetBool srv;
    srv.request.data = true;
    if (client.call(srv))
    {
        if (srv.response.success)
            ROS_INFO("Start Record");
        else
            ROS_INFO("fail record");
    }
    else
    {
        ROS_ERROR("Failed to call service record_service");
    }
}
void MainWindow::on_pushButton_voice_cmd_stop_clicked()
{
    ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>("record_service");
    std_srvs::SetBool srv;
    srv.request.data = false;
    if (client.call(srv))
    {
        if (srv.response.success)
        {
            ROS_INFO("Stop record");
            string str = srv.response.message;
            ROS_INFO("v2t:%s", str.c_str());

            /*test correct*/
            std::map<string, string> text_correct = { { "一腳", "椅腳" }, { "已面", "椅面" }, { "一面", "椅面" }, { "放行", "方形" }, { "已被", "椅背" }, { "西沉", "吸塵" }, { "沙子", "砂紙" }, { "影片", "椅面" }, { "爸爸說", "把板手" } };
            for (const auto &s : text_correct)
            {
                str = replace_text(str, s.first, s.second);
            }
            ROS_INFO("final:%s", str.c_str());
            ui->textEdit_voice->setText(QString::fromStdString(str));
        }
        else
            ROS_INFO("fail stoping record");
    }
    else
    {
        ROS_ERROR("Failed to call service record_service");
    }
    ROS_INFO("\033[0;94m=====NLP Done=====\033[0m");

    on_pushButton_creatActionBase_clicked();
}

/*Text to Action base*/
int MainWindow::find_text(string text, string find_text)
{
    string::size_type index;
    index = text.find(find_text);
    if (index != std::string::npos)
    {
        int position = index / 2;
        return index;
    }

    return -1;
}
std::string MainWindow::replace_text(std::string str, std::string find, std::string replace)
{
    // ROS_INFO("find and replace:%s->%s", find, replace);
    int index = find_text(str, find);
    if (index == -1)
        return str;
    int size = find.size();
    str.replace(index, size, replace);
    // ROS_INFO("replace:%s->%s", find, replace);
    return str;
}
void MainWindow::on_pushButton_creatActionBase_clicked()
{
    std::string voiceCmd = ui->textEdit_voice->toPlainText().toStdString();
    if (voiceCmd == "")
    {
        QMessageBox::about(NULL, "Error", "No voice command");
        return;
    }
    ROS_INFO("%sVoice Cmd : %s%s", C094, C0, voiceCmd.c_str());

    if (voiceCmd == "好" || voiceCmd == "停")
    {
        Action = "Stop";
        InteractObject = "";
        Destination = "";
        clearItem(nlp_itemModel);
        addItem(nlp_itemModel, QString::fromStdString(Action), QString::fromStdString(InteractObject), QString::fromStdString(Destination));
        return;
    }
    if (voiceCmd == "放開")
    {
        Action = "Release";
        InteractObject = "";
        Destination = "";
        clearItem(nlp_itemModel);
        addItem(nlp_itemModel, QString::fromStdString(Action), QString::fromStdString(InteractObject), QString::fromStdString(Destination));
        return;
    }

    // NLP
    std::map<string, string> map_action = {
        { "給我", "Place" },
        { "拿", "Place" },
        { "放", "Place" },
        { "磨", "Snad" },
        { "吸", "Vacuum" },
        { "掃", "Swipe" },
        { "漆", "Paint" },
        { "扶", "Hold" }
    };
    std::map<string, string> map_object = {
        { "起子", "Screwdriver" },
        // { "板手", "Wrench" },
        { "板手", "Hex keys" },
        { "砂紙", "Sand paper" },
        { "刷子", "Brush" },
        { "吸塵器", "Vacuum cleaner" },
        { "木板", "Board" },
        { "椅子", "Chair" },
        { "三角椅面", "Triangle seat" },
        { "方形椅面", "Squre seat" },
        // { "椅面", "Chair seat" },
        { "椅腳", "Chair leg" },
        { "椅背", "Chair back" },
        { "螺絲", "Screw" }
    };
    std::map<string, string> map_destination = {
        { "這裡", "Here" },
        { "桌", "Table" },
        { "木板", "Board" },
        { "回去", "Tool space" }
    };

    std::map<string, string> omit_action_fromDes = { { "Hand", "Place" } };
    std::map<string, string> omit_object_fromAct = { { "Snad", "Sand paper" }, { "Vacuum", "Vacuum cleaner" }, { "Paint", "Brush" } };
    std::map<string, string> omit_destination_fromObj = { {} };
    Action = "";
    InteractObject = "";
    Destination = "";


    // Action
    int index_find = -1;
    int size_find = 0;
    for (const auto &s : map_action)
    {
        int index = find_text(voiceCmd, s.first);
        if (index >= 0)
        {
            Action = s.second;
            int size = s.first.size();
            index_find = index;
            size_find = size;
            ROS_INFO("%sfind act : %s%s", C094, C0, s.first.c_str());
            break;
        }
    }
    //省略 動作
    if (index_find >= 0)
    {
        //動作不做刪除
        string replace = "";
        replace += "[act]";
        string tmpT = voiceCmd;
        tmpT.replace(index_find, size_find, replace);
        // ROS_INFO("%s", voiceCmd.c_str());
    }
    else if (Action == "")//省略 目標
    {
        Action = "Place";
        // ROS_INFO("%s", voiceCmd.c_str());
    }

    // Destination
    index_find = -1;
    size_find = 0;
    for (const auto &s : map_destination)
    {
        int index = find_text(voiceCmd, s.first);
        if (index >= 0)
        {
            Destination = s.second;
            int size = s.first.size();
            index_find = index;
            size_find = size;
            ROS_INFO("%sfind des : %s%s", C094, C0, s.first.c_str());
            break;
        }
    }

    if (index_find >= 0)
    {
        string replace = "";
        replace += "[des]";
        voiceCmd.replace(index_find, size_find, replace);
        // ROS_INFO("%s", voiceCmd.c_str());
    }
    else if (Destination == "")//省略 目標
    {
        Destination = "Hand";
        // ROS_INFO("%s", voiceCmd.c_str());
    }

    // interact
    index_find = -1;
    size_find = 0;
    for (const auto &s : map_object)
    {
        int index = find_text(voiceCmd, s.first);
        if (index >= 0)
        {
            InteractObject = s.second;
            int size = s.first.size();
            index_find = index;
            size_find = size;
            ROS_INFO("%sfind obj : %s%s", C094, C0, s.first.c_str());
            break;
        }
    }
    if (index_find >= 0)
    {
        string replace = "";
        replace += "[obj]";
        voiceCmd.replace(index_find, size_find, replace);
        // ROS_INFO("%s", voiceCmd.c_str());
    }
    else if (InteractObject == "")//省略 互動物件
    {
        if (Action == "Place")//當沒有互動物件時 那一定是拿 那個唯一物件，但先被Destination搶走。
        {
            InteractObject = Destination;
            Destination = "Hand";
            // ROS_INFO("%s", voiceCmd.c_str());
        }
        else
        {
            for (const auto &s : omit_object_fromAct)
            {
                int index = find_text(Action, s.first);
                if (index >= 0)
                {
                    InteractObject = s.second;
                }
            }
        }
    }
    clearItem(nlp_itemModel);
    addItem(nlp_itemModel, QString::fromStdString(Action), QString::fromStdString(InteractObject), QString::fromStdString(Destination));
}

/*NLP execute*/
void MainWindow::tm_grip_object(std::string obj)
{
    if (obj == "Brush")
    {
        addItem(msg_listModel, QString::fromStdString("[Move to] - Brush"));
        addItem(msg_listModel, QString::fromStdString("[Gripper] - grip"));
        addItem(msg_listModel, QString::fromStdString("[Move to] - home"));
    }
    else if (obj == "Sand paper")
    {
        addItem(msg_listModel, QString::fromStdString("[Move to] - Sand paper"));
        addItem(msg_listModel, QString::fromStdString("[Gripper] - grip"));
        addItem(msg_listModel, QString::fromStdString("[Move to] - home"));
    }
    else if (obj == "Triangle seat")
    {
        addItem(msg_listModel, QString::fromStdString("[Move to] - Triangle seat"));
        addItem(msg_listModel, QString::fromStdString("[Gripper] - grip"));
        addItem(msg_listModel, QString::fromStdString("[Move to] - home"));
        // ROS_INFO("\033[0;42m[Grip Tri-seat]\033[0m");
    }
    else if (obj == "Hex keys")
    {
        addItem(msg_listModel, QString::fromStdString("[Move to] - Hex keys"));
        addItem(msg_listModel, QString::fromStdString("[Gripper] - grip"));
        addItem(msg_listModel, QString::fromStdString("[Move to] - home"));
        // ROS_INFO("\033[0;42m[Grip Tri-seat]\033[0m");
    }
    else if (obj == "Chair leg")
    {
        addItem(msg_listModel, QString::fromStdString("[Move to] - Chair leg"));
        addItem(msg_listModel, QString::fromStdString("[Gripper] - grip"));
        addItem(msg_listModel, QString::fromStdString("[Move to] - home"));
        // ROS_INFO("\033[0;42m[Grip Tri-seat]\033[0m");
    }

    else if (obj == "Vacuum cleaner")
    {
        tm.moveTo(TmJoint(-27.52, -12.44, -58.81, -90.08, 20.23, -163.63, "deg"), 100, true, true); // out safe
        tm.moveTo(TmJoint(-18.11, -25.40, -38.77, -102.67, 29.30, -169.71, "deg"), 100, true, true);// up
        tm.moveTo(TmJoint(-18.13, -20.83, -57.07, -88.90, 29.27, -169.67, "deg"), 100, true, true); // grip
        sendGrip(true);
        sleep(3);
        tm.moveTo(TmJoint(-18.11, -25.40, -38.77, -102.67, 29.30, -169.71, "deg"), 100, true, true);// up
        tm.moveTo(TmJoint(-27.52, -12.44, -58.81, -90.08, 20.23, -163.63, "deg"), 100, true, true); // out safe
    }
    else
    {
        QMessageBox::about(NULL, "Error", "Unknow objet!");
    }
}
void MainWindow::tm_place_object(std::string obj)
{
    if (obj == "Brush")
    {
        addItem(msg_listModel, QString::fromStdString("[Move to] - Brush"));
        addItem(msg_listModel, QString::fromStdString("[Gripper] - release"));
        addItem(msg_listModel, QString::fromStdString("[Move to] - home"));
    }
    else if (obj == "Sand paper")
    {
        addItem(msg_listModel, QString::fromStdString("[Move to] - Sand paper"));
        addItem(msg_listModel, QString::fromStdString("[Gripper] - release"));
        addItem(msg_listModel, QString::fromStdString("[Move to] - home"));
    }
    else if (obj == "Triangle seat")
    {
        addItem(msg_listModel, QString::fromStdString("[Move to] - Triangle seat"));
        addItem(msg_listModel, QString::fromStdString("[Gripper] - release"));
        addItem(msg_listModel, QString::fromStdString("[Move to] - home"));
        // ROS_INFO("\033[0;42m[Grip Tri-seat]\033[0m");
    }
    else if (obj == "Hex keys")
    {
        addItem(msg_listModel, QString::fromStdString("[Move to] - Hex keys"));
        addItem(msg_listModel, QString::fromStdString("[Gripper] - release"));
        addItem(msg_listModel, QString::fromStdString("[Move to] - home"));
        // ROS_INFO("\033[0;42m[Grip Tri-seat]\033[0m");
    }
    else if (obj == "Chair leg")
    {
        addItem(msg_listModel, QString::fromStdString("[Move to] - Chair leg"));
        addItem(msg_listModel, QString::fromStdString("[Gripper] - release"));
        addItem(msg_listModel, QString::fromStdString("[Move to] - home"));
        // ROS_INFO("\033[0;42m[Grip Tri-seat]\033[0m");
    }
    else if (obj == "Vacuum cleaner")
    {
        // sendCMD("PTP(\"JPP\",-27.52,-12.44,-58.81,-90.08,20.23,-163.63,100,200,0,false)");// out safe
        // waitTM(1);
        // sendCMD("PTP(\"JPP\",-18.11,-25.40,-38.77,-102.67,29.30,-169.71,100,200,0,false)");// up
        // waitTM(1);
        // sendCMD("PTP(\"JPP\",-18.13,-20.83,-57.07,-88.90,29.27,-169.67,100,200,0,false)");// grip
        // waitTM(1);
        // sendGrip(false);
        // sleep(1);
        // sendCMD("PTP(\"JPP\",-18.11,-25.40,-38.77,-102.67,29.30,-169.71,100,200,0,false)");// up
        // waitTM(1);
        // sendCMD("PTP(\"JPP\",-27.52,-12.44,-58.81,-90.08,20.23,-163.63,100,200,0,false)");// out safe
        // waitTM(1);
    }
    else
    {
        QMessageBox::about(NULL, "Error", "Unknow objet!");
    }
}


void MainWindow::on_btn_tm_execute_nlp_clicked()
{
    clearItem(msg_listModel);

    addItem(msg_listModel, QString::fromStdString("=====Execute====="));
    if (Action == "Stop")
    {
        servoing = false;
        addItem(msg_listModel, "Stop");
        sendCMD("StopAndClearBuffer()");
        return;
    }
    else if (Action == "Release")
    {
        servoing = false;
        addItem(msg_listModel, QString::fromStdString("[Gripper] - release"));
        GrippingObject = "";
        ui->label_grippingObject->setText(setStringColor(QString::fromStdString(GrippingObject), "#c25555"));
        return;
    }

    // Action     InteractObject     Destination
    clearItem(msg_listModel);

    /*Gripping Objict*/
    if (GrippingObject != "" && InteractObject != GrippingObject)
    {
        addItem(msg_listModel, QString::fromStdString("==Place " + GrippingObject + "=="));
        tm_place_object(GrippingObject);
        GrippingObject = "";
        ui->label_grippingObject->setText("");
    }
    if (GrippingObject != InteractObject)
    {
        addItem(msg_listModel, QString::fromStdString("==Pick " + InteractObject + "=="));
        tm_grip_object(InteractObject);
        GrippingObject = InteractObject;
        ui->label_grippingObject->setText(setStringColor(QString::fromStdString(GrippingObject), "#c25555"));
    }

    /*Action*/
    if (Action == "Place")
    {
        if (Destination == "Tool space")
        {
            tm_place_object(GrippingObject);
            GrippingObject = "";
            ui->label_grippingObject->setText(setStringColor(QString::fromStdString(GrippingObject), "#c25555"));
        }
        else if (Destination == "Hand")
        {
            addItem(msg_listModel, QString::fromStdString("==Place to Hand=="));
            addItem(msg_listModel, QString::fromStdString("[Move to] - hand"));
            // cv::Point2f p = getTrackPoint_blue();
            // sendCMD("PTP(\"CPP\"," + std::to_string(p.x * 1000) + "," + std::to_string(p.y * 1000) + ",300,125,0,-45,100,200,0,false)");// mm-deg up
            // waitTM(2);
            addItem(msg_listModel, QString::fromStdString("[Gripper] - release"));
            // sendGrip(false);
            // sleep(1);
            addItem(msg_listModel, QString::fromStdString("[Move to] - home"));
            // sendCMD("PTP(\"JPP\",-18.46,8.62,-89.74,-135.47,72.34,-193.04,100,200,0,false)");// coner out safe
            // waitTM(1);
            GrippingObject = "";
            ui->label_grippingObject->setText(setStringColor(QString::fromStdString(GrippingObject), "#c25555"));
        }
    }
    else if (Action == "Snad")
    {
        if (Destination == "Here")
        {
            addItem(msg_listModel, QString::fromStdString("[Sand]"));
            // cv::Point2f p = getTrackPoint_blue();
            // p.x -= 0.03;
            // p.y += 0.03;
            // sendCMD("PTP(\"CPP\"," + std::to_string((p.x * 1000) + 30) + "," + std::to_string(p.y * 1000) + ",250,125,0,-45,100,200,0,false)");// mm-deg up
            // sleep(3);
            // sendCMD("PTP(\"CPP\"," + std::to_string((p.x * 1000) - 30) + "," + std::to_string(p.y * 1000) + ",250,125,0,-45,100,200,0,false)");// mm-deg up
            // // sleep(0.5);
            // sendCMD("PTP(\"CPP\"," + std::to_string((p.x * 1000) - 30) + "," + std::to_string(p.y * 1000) + ",250,125,0,-45,100,200,0,false)");// mm-deg up
            // // sleep(0.5);
            // sendCMD("PTP(\"CPP\"," + std::to_string((p.x * 1000) + 30) + "," + std::to_string(p.y * 1000) + ",250,125,0,-45,100,200,0,false)");// mm-deg up
            // // sleep(0.5);
            // sendCMD("PTP(\"CPP\"," + std::to_string((p.x * 1000) - 30) + "," + std::to_string(p.y * 1000) + ",250,125,0,-45,100,200,0,false)");// mm-deg up
            // // sleep(0.5);
            // sendCMD("PTP(\"CPP\"," + std::to_string((p.x * 1000) + 30) + "," + std::to_string(p.y * 1000) + ",250,125,0,-45,100,200,0,false)");// mm-deg up
            // // sleep(0.5);
        }
    }
    else if (Action == "Vacuum")
    {
        sendCMD("PTP(\"JPP\",-28.00,-8.16,-89.18,-30.65,-14.29,-84.75,100,200,0,false)");// out safe
        waitTM(1);
        cv::Point2f p = getTrackPoint_blue();
        p.x -= 0.09;
        p.y += 0.09;
        sendCMD("PTP(\"CPP\"," + std::to_string(p.x * 1000) + "," + std::to_string(p.y * 1000) + ",302,105.48,-43.41,-29.67,100,200,0,false)");// mm-deg
        sleep(2);
        // sendCMD("PTP(\"CPP\"," + std::to_string(p.x * 1000) + "," + std::to_string(p.y * 1000) + ",320,105.48,-43.41,-29.67,100,200,0,false)"); // mm-deg
        waitTM(1);
        // Track
        // thread_TM_servoOn("/track/blue");
    }
    else if (Action == "Paint")
    {
        if (Destination == "Here")
        {
        }
    }
    else if (Action == "Hold")
    {
        if (Destination == "Here")
        {
            addItem(msg_listModel, setStringColor(QString::fromStdString("==Hold Here=="), "#696318"));
            addItem(msg_listModel, QString::fromStdString("[Move to] - here"));
        }
        else if (Destination == "Hand")
        {
            addItem(msg_listModel, setStringColor(QString::fromStdString("==Hold Hand=="), "#696318"));
            addItem(msg_listModel, QString::fromStdString("[Move to] - hand"));
        }
    }
    else
    {
        QMessageBox::about(NULL, "Error", "Unknow action!");
    }
}
