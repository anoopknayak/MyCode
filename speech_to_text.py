#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import speech_recognition as sr


if __name__ == "__main__":

    rospy.init_node('speech', anonymous=True)
    r = sr.Recognizer()
    m = sr.Microphone()

    use_api = True


    def convert(sentence):
        # we need some special handling here to correctly print unicode characters to standard output
        if str is bytes:  # this version of Python uses bytes for strings (Python 2)
            return format(sentence).encode("utf-8")
        else:  # this version of Python uses unicode for strings (Python 3+)
            return format(sentence)

    def local_speech(audio):
        recognised_text = convert(r.recognize_sphinx(audio))
        rospy.loginfo("You said (Internal): %s", recognised_text)
        return recognised_text

    def google_speech(audio):
        try:
            # recognize speech using Google Speech Recognition
            recognised_text = convert(r.recognize_google(audio))
            rospy.loginfo("You said (Google): %s", recognised_text)
            return recognised_text

        except sr.UnknownValueError:
            rospy.loginfo("Oops! Didn't catch that")

        except sr.RequestError as e:
            rospy.loginfo("Uh oh! Couldn't request results from Google Speech Recognition service")
            return local_speech(audio)


    try:
        rospy.loginfo("A moment of silence, please...")
        with m as source:
            r.adjust_for_ambient_noise(source)
        rospy.loginfo("Set minimum energy threshold to {}".format(r.energy_threshold))

        # Publish the String message to the speech_to_text topic
        string_pub = rospy.Publisher('speech_to_text', String, queue_size=5)

        while not rospy.is_shutdown():
            recognised = None
            rospy.loginfo("Say something!")
            with m as source:
                speech = r.listen(source)

            rospy.loginfo("Got it! Now to recognize it...")

            if use_api:
                recognised = google_speech(speech)
            else:
                recognised = local_speech(speech)

            if recognised is not None:
                text = String()
                text.data = recognised
                string_pub.publish(text)

    except rospy.ROSInterruptException:
        pass
