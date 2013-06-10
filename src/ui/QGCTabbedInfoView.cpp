#include "QGCTabbedInfoView.h"

QGCTabbedInfoView::QGCTabbedInfoView(QWidget *parent) : QWidget(parent)
{
    ui.setupUi(this);
    actionsWidget = new UASActionsWidget(this);
    quickView = new UASQuickView(this);
    rawView = new UASRawStatusView(this);
    ui.tabWidget->addTab(quickView,"Quick");
    ui.tabWidget->addTab(actionsWidget,"Actions");
    ui.tabWidget->addTab(rawView,"Status");
}
void QGCTabbedInfoView::addSource(MAVLinkDecoder *decoder)
{
    m_decoder = decoder;
    rawView->addSource(decoder);
    quickView->addSource(decoder);
}

QGCTabbedInfoView::~QGCTabbedInfoView()
{
}