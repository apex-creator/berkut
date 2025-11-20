const common = require("../.project/common.js");

const docs_list= [
    "docs/Low_Power_Visualizer_User_Guide.pdf",
    "docs/Motion_Presence_Detection_Demo_Group_Tracker_Tuning_Guide.pdf",
    "docs/Motion_Presence_Detection_Demo_Tuning_Guide.pdf"
]

function getDocsList() {
    return docs_list;
}

module.exports = {
    getDocsList
};