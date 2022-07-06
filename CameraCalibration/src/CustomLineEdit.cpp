#include "CustomLineEdit.h"

void CustomLineEdit::focusInEvent(QFocusEvent *event)
{
    m_originalValue = text();
    QLineEdit::focusOutEvent(event);
}

void CustomLineEdit::focusOutEvent(QFocusEvent *event)
{
    QLineEdit::focusOutEvent(event);
    if (text() != m_originalValue && !hasAcceptableInput())
    {
        setText(m_originalValue);
        setFocus();
        emit validationError();
    }
}


