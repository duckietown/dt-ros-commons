import asyncio
import json
import re
import time
from abc import ABC, abstractmethod
from asyncio import TimeoutError
from enum import Enum
from json import JSONEncoder
from typing import Any, List, Optional

import rospy
from dtps import DTPSContext
from dtps_http import RawData
from duckietown_messages.standard.dictionary import Dictionary
from duckietown_messages.standard.header import Header
from std_srvs.srv import Trigger, TriggerResponse

DOCS_SUB_DOMAIN = "docs"
DOCS_RELEASE_DISTRO = "ente"
DOCS_BASE_URL = (
    f"https://{DOCS_SUB_DOMAIN}.duckietown.com/{DOCS_RELEASE_DISTRO}"
    "/opmanual-duckiebot/operations/dashboard"
    "/user_hardware_testing_tools.html#{url_section_name}"
)
TIMEOUT_BUFFER = 3


class HardwareTestJsonParamType(Enum):
    """Type constants, so receiving side knows how to parse/use the values"""

    # basic string
    STRING = "string"
    # base64 encode image
    BASE64 = "base64"
    # a html block (as a string)
    HTML = "html"
    # ROS Topic related info, i.e. name and type. for the "stream" option below
    TOPIC_INFO = "topic_info"

    # --- Highest level response types ---
    # results of a completed test
    OBJECT = "object"
    # instructions to connect to a live stream from a topic
    STREAM = "stream"


class EnumJSONEncoder(JSONEncoder):
    """Used to encode the HardwareTestJsonParamType(Enum)"""

    def default(self, obj):
        if isinstance(obj, Enum):
            return obj.value
        return super().default(obj)


class AbstractHardwareTestROSInterface(ABC):
    _in_queue: DTPSContext
    _node: Any
    _out_queue: DTPSContext
    _result: Optional[dict]
    test_id: str
    test_in_queue: Optional[DTPSContext]
    test_timeout: int

    def __init__(self, node: Any, test_id: str, test_in_queue: Optional[DTPSContext] = None, test_timeout: int = 60, service_identifier: str = "test") -> None:
        self._node = node
        self.test_in_queue = test_in_queue
        self.test_id = test_id
        self.test_timeout = test_timeout
        self._result = None
        # test services
        rospy.Service(f"~{service_identifier}/description", Trigger, self.cb_description)
        rospy.Service(f"~{service_identifier}/run", Trigger, self.cb_run_test)

    async def cb_data(self, raw_data: RawData) -> None:
        dictionary: Dictionary = Dictionary.from_rawdata(raw_data)
        self._result = dictionary.data

    def cb_description(self, _):
        """The test description service response"""
        return self.format_response_object(
            success=True,
            lst_blocks=self.test_description(),
        )

    def cb_run_test(self, _):
        data = {
            "test_id": self.test_id,
            "test_timeout": self.test_timeout
        }
        data = self.get_test_data(data)
        timestamp = time.time()
        header = Header(timestamp=timestamp)
        dictionary = Dictionary(header=header, data=data)
        raw_data = dictionary.to_rawdata()
        publish_coroutine = self.test_in_queue.publish(raw_data)
        self._node.loginfo(f"[{self.test_id}] Running test...")
        try:
            asyncio.run(publish_coroutine)
        except TimeoutError:
            self._node.logerr(f"[{self.test_id}] Error publishing to {self.test_in_queue}.")
            return self.format_response_object(success=False, lst_blocks=[])
        sleep_duration = 0.1
        counter = 0
        while self._result is None:
            if counter > self.test_timeout + TIMEOUT_BUFFER:
                self._node.logerr(f"[{self.test_id}] Test timed out.")
                return self.format_response_object(success=False, lst_blocks=[])
            if round(counter, 1) % 1 == 0:
                self._node.loginfo(f"[{self.test_id}] Waiting for test result... ({round(counter)}/{self.test_timeout + TIMEOUT_BUFFER} seconds)")
            time.sleep(sleep_duration)
            counter += sleep_duration
        self._node.loginfo(f"[{self.test_id}] Test result received.")
        response_object = self.format_response_object(
            success=self._result["success"],
            lst_blocks=self._result["lst_blocks"],
        )
        self._result = None
        return response_object

    @staticmethod
    def format_obj(key: str, value_type: HardwareTestJsonParamType, value: str) -> dict:
        return {
            "key": key,
            "type": value_type,
            "value": value,
        }

    @staticmethod
    def format_response_object(success: bool, lst_blocks):
        ret_obj = {"type": HardwareTestJsonParamType.OBJECT, "parameters": []}
        for block in lst_blocks:
            ret_obj["parameters"].append(block)

        return TriggerResponse(
            success=success,
            message=json.dumps(ret_obj, cls=EnumJSONEncoder),
        )

    @staticmethod
    def format_response_stream(
        success: bool,
        test_topic_name: str,
        test_topic_type: str,
        lst_blocks,
    ):
        ret_obj = {"type": HardwareTestJsonParamType.STREAM, "parameters": []}
        for block in lst_blocks:
            ret_obj["parameters"].append(block)

        ret_obj["parameters"].append(
            AbstractHardwareTestROSInterface.format_obj(
                key="test_topic_name",
                value_type=HardwareTestJsonParamType.TOPIC_INFO,
                value=test_topic_name,
            )
        )

        ret_obj["parameters"].append(
            AbstractHardwareTestROSInterface.format_obj(
                key="test_topic_type",
                value_type=HardwareTestJsonParamType.TOPIC_INFO,
                value=test_topic_type,
            )
        )

        return TriggerResponse(
            success=success,
            message=json.dumps(ret_obj, cls=EnumJSONEncoder),
        )

    @abstractmethod
    def get_test_data(self, data: dict) -> dict:
        """Get test data to be sent to the client"""
        pass

    @staticmethod
    def html_util_ul(lst_items: List[str]) -> str:
        """Helper function to add a list of html code to an unordered list"""
        ret = ["<ul>"]
        for item in lst_items:
            ret.append("<li>" + item + "</li>")
        ret.append("</ul>")
        return "".join(ret)

    def test_description(self) -> List:
        """Test descriptions"""
        return [
            AbstractHardwareTestROSInterface.format_obj(
                key="Preparation",
                value_type=HardwareTestJsonParamType.HTML,
                value=self.test_description_preparation(),
            ),
            AbstractHardwareTestROSInterface.format_obj(
                "Expected Outcomes",
                HardwareTestJsonParamType.HTML,
                self.test_description_expectation(),
            ),
            AbstractHardwareTestROSInterface.format_obj(
                "How to run",
                HardwareTestJsonParamType.HTML,
                self.test_description_running(),
            ),
            AbstractHardwareTestROSInterface.format_obj(
                "Getting help with issues",
                HardwareTestJsonParamType.HTML,
                self.test_description_collapsable_info_panel(),
            ),
        ]

    def test_description_collapsable_info_panel(self):
        """Use Bootstrap3 collapsable panel for Logs and FAQs sections"""

        # convert arbitrary string ID to a valid html element ID
        id_str = re.sub(r'[^a-zA-Z0-9-_:]', '_', self.test_id)
        panel_id = f"getting-help-panel-{id_str}"

        contents = "".join([
            "Demo videos and FAQs",
            self.test_description_link_to_docs(),
            "Logs Gathering (in case of errors)",
            self.test_description_log_gather(),
        ])

        return f"""
        <ul><li><button class="btn" data-toggle="collapse" data-target="#{panel_id}">
        Click to toggle the information</button></li></ul>
        <div class="panel panel-default collapse" id="{panel_id}">
            <div class="panel-body">
                {contents}
            </div>
        </div>
        """

    @abstractmethod
    def test_description_expectation(self) -> str:
        """Expected outcome(s) and/or how to determine if a test was successful"""
        pass

    @abstractmethod
    def test_description_preparation(self) -> str:
        """Preparation before running. E.g. put the Duckiebot upside down"""
        pass

    def test_description_link_to_docs(self) -> str:
        """Link to official documentation about the Hardware Tests"""
        url_videos = DOCS_BASE_URL.format(url_section_name="demos-of-the-hardware-tests")
        url_faqs = DOCS_BASE_URL.format(url_section_name="faqs-reporting-problems-getting-help")
        return self.html_util_ul([
            f"<a href='{url_videos}'><strong>How-to</strong> series videos</a>",
            f"<a href='{url_faqs}'>FAQs and getting help</a>",
        ]) + "<p style='font-size: 8pt'>(In case of broken links, please report on the Duckietown Slack.)</p>"

    def test_description_log_gather(self) -> str:
        """How to gather logs before reporting"""
        return self.html_util_ul(
            [
                "On your laptop, run the following command to save the logs.",
                "Replace the <code>[path/to/save]</code> to the directory path where you would like to save the logs.",
                "<code>docker -H [ROBOT_NAME].local logs duckiebot-interface > [path/to/save/]logs-db-iface.txt</code>",
            ]
        )

    def test_description_running(self) -> str:
        """Actual steps to run the test"""
        # default: just click the "Run test" button
        return self.html_util_ul(
            ["Click on the <strong>Run the test</strong> button below."]
        )
