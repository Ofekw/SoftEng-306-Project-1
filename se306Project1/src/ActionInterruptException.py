__author__ = 'patrick'

class ActionInterruptException (BaseException):

    def _init_(self, message):
        super(ActionInterruptException, self, message)._init_()