import os
import sys
from reportlab.lib.pagesizes import A4
from reportlab.lib import colors
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image, KeepTogether, PageBreak, HRFlowable
)
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfbase.pdfmetrics import registerFontFamily
from reportlab.pdfgen import canvas

# 1. Font Kaydı (Türkçe karakter desteği için Arial)
pdfmetrics.registerFont(TTFont('Arial', r'C:\Windows\Fonts\arial.ttf'))
pdfmetrics.registerFont(TTFont('Arial-Bold', r'C:\Windows\Fonts\arialbd.ttf'))
pdfmetrics.registerFont(TTFont('Arial-Italic', r'C:\Windows\Fonts\ariali.ttf'))
pdfmetrics.registerFont(TTFont('Arial-BoldItalic', r'C:\Windows\Fonts\arialbi.ttf'))
registerFontFamily('Arial', normal='Arial', bold='Arial-Bold', italic='Arial-Italic', boldItalic='Arial-BoldItalic')

class NumberedCanvas(canvas.Canvas):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._saved_page_states = []

    def showPage(self):
        self._saved_page_states.append(dict(self.__dict__))
        self._startPage()

    def save(self):
        num_pages = len(self._saved_page_states)
        for state in self._saved_page_states:
            self.__dict__.update(state)
            self.draw_page_decorations(num_pages)
            super().showPage()
        super().save()

    def draw_page_decorations(self, page_count):
        self.saveState()
        self.setFont("Arial", 8)
        self.setFillColor(colors.HexColor("#718096"))
        
        # Header (Sayfa 1 hariç)
        if self._pageNumber > 1:
            self.drawString(45, 805, "TÜBİTAK UZAY — GZGSG Lens Distorsiyonu Ölçüm ve Kalibrasyon Raporu")
            self.drawRightString(550, 805, "05.11.2026")
            self.setStrokeColor(colors.HexColor("#CBD5E0"))
            self.setLineWidth(0.5)
            self.line(45, 797, 550, 797)

        # Footer
        self.setStrokeColor(colors.HexColor("#CBD5E0"))
        self.setLineWidth(0.5)
        self.line(45, 45, 550, 45)
        self.drawString(45, 32, "TÜBİTAK UZAY | Opto-Mekanik Sistemler Laboratuvarı — Gizli / Kurum İçi")
        self.drawRightString(550, 32, f"Sayfa {self._pageNumber} / {page_count}")
        self.restoreState()

def build_pdf(output_path):
    doc = SimpleDocTemplate(
        output_path,
        pagesize=A4,
        leftMargin=45,
        rightMargin=45,
        topMargin=54,
        bottomMargin=54
    )

    styles = getSampleStyleSheet()
    
    # Özel Stiller
    title_style = ParagraphStyle(
        'DocTitle',
        fontName='Arial-Bold',
        fontSize=18,
        leading=22,
        textColor=colors.HexColor('#1A365D'),
        spaceAfter=4
    )
    
    subtitle_style = ParagraphStyle(
        'DocSubTitle',
        fontName='Arial',
        fontSize=11,
        leading=15,
        textColor=colors.HexColor('#4A5568'),
        spaceAfter=12
    )

    h1_style = ParagraphStyle(
        'Heading1_Custom',
        fontName='Arial-Bold',
        fontSize=13,
        leading=17,
        textColor=colors.HexColor('#1A365D'),
        spaceBefore=14,
        spaceAfter=6,
        keepWithNext=True
    )

    h2_style = ParagraphStyle(
        'Heading2_Custom',
        fontName='Arial-Bold',
        fontSize=10.5,
        leading=14,
        textColor=colors.HexColor('#2B6CB0'),
        spaceBefore=10,
        spaceAfter=4,
        keepWithNext=True
    )

    body_style = ParagraphStyle(
        'Body_Custom',
        fontName='Arial',
        fontSize=9,
        leading=13.5,
        textColor=colors.HexColor('#2D3748'),
        spaceAfter=6
    )

    bullet_style = ParagraphStyle(
        'Bullet_Custom',
        fontName='Arial',
        fontSize=9,
        leading=13,
        textColor=colors.HexColor('#2D3748'),
        leftIndent=15,
        firstLineIndent=-10,
        spaceAfter=3
    )

    code_style = ParagraphStyle(
        'Code_Custom',
        fontName='Arial',
        fontSize=8,
        leading=11.5,
        textColor=colors.HexColor('#1A202C')
    )

    caption_style = ParagraphStyle(
        'Caption',
        fontName='Arial-Italic',
        fontSize=8,
        leading=11,
        textColor=colors.HexColor('#4A5568'),
        alignment=1, # Center
        spaceBefore=4,
        spaceAfter=8
    )

    story = []

    # BAŞLIK ALANI
    story.append(Paragraph("TÜBİTAK UZAY — GZGSG PROJESİ", ParagraphStyle('TopHeader', fontName='Arial-Bold', fontSize=9, textColor=colors.HexColor('#C53030'), leading=11, spaceAfter=2)))
    story.append(Paragraph("Lens Distorsiyonu ve Kamera Kalibrasyon Sistemi", title_style))
    story.append(Paragraph("<b>Teknik Geliştirme, Fiziksel Kurulum Analizi ve 3B [R | T] Modeli Raporu</b>", subtitle_style))
    story.append(HRFlowable(width="100%", thickness=1.5, color=colors.HexColor('#1A365D'), spaceBefore=0, spaceAfter=8))

    # META BİLGİ TABLOSU
    meta_data = [
        [
            Paragraph("<b>Tarih:</b> 5 Kasım 2026", body_style),
            Paragraph("<b>Hazırlayan:</b> İdil Su Nokta", body_style),
            Paragraph("<b>Depo:</b> GZGSG-Distortion-Measurement", body_style)
        ],
        [
            Paragraph("<b>Kamera:</b> Allied Vision Manta G-419 (CMV4000 1\")", body_style),
            Paragraph("<b>Lens:</b> Basler 12mm f/2.0 (C-Mount)", body_style),
            Paragraph("<b>Kaynak:</b> HeNe (632.8 nm) + 150 µm Pinhole", body_style)
        ]
    ]
    meta_table = Table(meta_data, colWidths=[170, 165, 170])
    meta_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,-1), colors.HexColor('#EDF2F7')),
        ('BOX', (0,0), (-1,-1), 0.5, colors.HexColor('#CBD5E0')),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('TOPPADDING', (0,0), (-1,-1), 4),
        ('BOTTOMPADDING', (0,0), (-1,-1), 4),
        ('LEFTPADDING', (0,0), (-1,-1), 8),
        ('RIGHTPADDING', (0,0), (-1,-1), 8),
    ]))
    story.append(meta_table)
    story.append(Spacer(1, 10))

    # 1. BÖLÜM: YÖNETİCİ ÖZETİ
    story.append(Paragraph("1. Yönetici Özeti (Executive Summary)", h1_style))
    summary_text = (
        "Bu rapor, 5 Kasım 2026 tarihinde opto-mekanik kalibrasyon sisteminde gerçekleştirilen yazılımsal altyapı "
        "dönüşümünü, görüntü işleme filtreleme iyileştirmelerini ve matematiksel modelleme revizyonunu özetlemektedir. "
        "Laboratuvar testlerinde alınan 30×30 ve 15×15 tarama sonuçlarında tespit edilen <b>perspektif yamukluğu (trapezoid/keystone)</b>, "
        "<b>tanjansiyel distorsiyon anomalisi (p₁ = -0.1014)</b> ve <b>odak uzaklığı sapması (fx ≈ 1196 px vs. teorik 2182 px)</b> "
        "problemlerinin kök nedeni fiziksel incelemelerle aydınlatılmıştır. "
        "Basitleştirilmiş açısal formülasyondan, kameranın masadaki duruşunu ve pinhole mesafesini doğrudan hesaba katan "
        "<b>Tam 3 Boyutlu Dışsal [R | T] + Brown-Conrady Projeksiyon Modeline</b> geçilmiştir."
    )
    story.append(Paragraph(summary_text, body_style))

    # 2. BÖLÜM: YAZILIMSAL VE ALGORİTMİK GELİŞTİRMELER
    story.append(Paragraph("2. Yazılımsal ve Algoritmik Geliştirmeler", h1_style))
    story.append(Paragraph("A. Adaptif Izgara Takibi ve Hata Toleransı (`AdaptiveGridTracker` & `NOT_DETECTED`)", h2_style))
    story.append(Paragraph("• <b>Üç Kademeli Arama Tahmini:</b> Satır içi adımlarda önceki sütun koordinatından ilerleme, satır atlamalarında (daktilo dönüşü) önceki satırın Y ortalamasını çıpalama ve ilk noktayı ($P_{01}$) mutlak referans alma stratejisiyle arama kutusu kilitlendi.", bullet_style))
    story.append(Paragraph("• <b>Global Fallback (Küresel Yakalayıcı):</b> Yerel ROI penceresinin kaçırdığı durumlarda tüm görüntüde bağlı bileşen analizi (connected components) ile lazer beneği bulunup arama kutusu otomatik olarak lazerin üzerine çekildi.", bullet_style))
    story.append(Paragraph("• <b>NOT_DETECTED Fail-Safe Mimarisi:</b> Lazer noktasının tespit edilemediği durumlarda rastgele koordinat üretmek yerine o adım <code>NaN</code> olarak işaretlenip atlandı. Böylece aykırı bir noktanın tüm kalibrasyon fitini bozması engellendi.", bullet_style))
    story.append(Paragraph("• <b>Tek Seferlik Pozlama Kalibrasyonu:</b> Tarama başlamadan önce kamera pozlaması bir kez optimize edilip kilitlendi (`calibrate_exposure_once`). Ölçüm adımlarındaki gecikmeler kaldırılarak tarama süresi 3 kat hızlandırıldı.", bullet_style))

    story.append(Paragraph("B. Git Depo Mimarisi ve Modüler Dizin Standardizasyonu", h2_style))
    story.append(Paragraph("Proje dizini (`GZGSG-Distortion-Measurement`) temiz bir mühendislik mimarisine kavuşturuldu:", body_style))
    story.append(Paragraph("• <b>docs/:</b> Proje raporları (.docx, .pdf) ve sunum dosyaları (.pptx) bu klasöre toplandı.", bullet_style))
    story.append(Paragraph("• <b>archive/:</b> Ana dizindeki uzantısız eski betikler (.py uzantısıyla) arşiv klasörüne kaldırıldı.", bullet_style))
    story.append(Paragraph("• <b>data/ & measurements/:</b> Ham ölçüm CSV dosyaları düzenli bir dizine yerleştirildi.", bullet_style))
    story.append(Paragraph("• <b>requirements.txt:</b> Bağımlılıklar (vmbpy, opencv-python, scipy, pyserial, numpy, matplotlib, pandas, openpyxl) sürümleriyle tanımlandı.", bullet_style))
    story.append(Paragraph("• <b>README.md:</b> Çift dilli (Türkçe & İngilizce) tam dokümantasyon, kurulum adımları ve parametre kılavuzu hazırlandı.", bullet_style))

    story.append(Spacer(1, 8))

    # 3. BÖLÜM: FİZİKSEL KURULUM VE DONANIM DOĞRULAMASI
    story.append(Paragraph("3. Fiziksel Kurulum ve Donanım Doğrulaması", h1_style))
    setup_text = (
        "Önceki dokümanlarda yer alan <i>'kamera ve lazer gimbal üzerinde monteli, duvara bakıyor'</i> varsayımının "
        "hatalı olduğu tespit edilmiş; laboratuvardaki gerçek kurulum fotoğraflarla doğrulanmıştır:<br/>"
        "• <b>Gimbal (Hareketli Kısım):</b> Üzerinde kamera bulunmamaktadır. 3D baskı kol ve Thorlabs optik tutucu ile "
        "<b>kolimatör + 150 µm pinhole</b> taşınmakta ve ışık kaynağı açısal/konumsal olarak hareket ettirilmektedir.<br/>"
        "• <b>Kamera (Sabit Kısım):</b> Gimbaldan bağımsız, masada optik yükseltici bacaklar (posts) üzerinde <b>SABİT</b> "
        "durmakta ve karşıdan gelen pinhole ışığına bakmaktadır.<br/>"
        "• <b>Çalışma Mesafesi:</b> Pinhole çıkışı ile lens ön yüzü arasındaki mesafe <b>~6.3 cm ile 12 cm</b> arasındadır."
    )
    story.append(Paragraph(setup_text, body_style))

    # Setup Fotoğrafları Tablosu
    img_pinhole = r'C:/Users/user/.gemini/antigravity/brain/89980c0c-86ba-4703-85a2-b58a21defbaf/.user_uploaded/media_1793878792080_23a7b38b.png'
    img_camera = r'C:/Users/user/.gemini/antigravity/brain/89980c0c-86ba-4703-85a2-b58a21defbaf/.user_uploaded/media_1793878960979_3d36e69e.png'
    
    if os.path.exists(img_pinhole) and os.path.exists(img_camera):
        im1 = Image(img_pinhole, width=240, height=130)
        im2 = Image(img_camera, width=240, height=130)
        img_table = Table([
            [im1, im2],
            [Paragraph("<b>Şekil 1:</b> Gimbal üzerindeki Thorlabs tutucu ve 150 µm pinhole.", caption_style),
             Paragraph("<b>Şekil 2:</b> Masada ayaklar üzerinde sabit Manta G-419 + Basler 12mm.", caption_style)]
        ], colWidths=[250, 250])
        img_table.setStyle(TableStyle([
            ('ALIGN', (0,0), (-1,-1), 'CENTER'),
            ('VALIGN', (0,0), (-1,-1), 'TOP'),
            ('BOTTOMPADDING', (0,0), (-1,-1), 2),
            ('TOPPADDING', (0,0), (-1,-1), 2),
        ]))
        story.append(img_table)

    story.append(PageBreak()) # SAYFA 2

    # 4. BÖLÜM: 30x30 KALİBRASYON BULGULARI VE KÖK NEDEN ANALİZİ
    story.append(Paragraph("4. 30×30 Kalibrasyon Bulguları ve Kök Neden Analizi", h1_style))
    analysis_intro = (
        "Bugün tamamlanan 30×30 tarama ölçümünde (856 geçerli nokta, 42 aykırı değer) elde edilen sonuçlar ve üretilen "
        "kırmızı ızgara grafiği detaylı incelenmiştir:"
    )
    story.append(Paragraph(analysis_intro, body_style))

    # Kırmızı Mesh Görseli ve Tablo
    img_mesh = r'C:/Users/user/.gemini/antigravity/brain/89980c0c-86ba-4703-85a2-b58a21defbaf/.user_uploaded/media_1793877812255_051fd6c3.jpg'
    
    table_calib_data = [
        ["Parametre", "Ölçülen Değer", "Teorik / Beklenen", "Fiziksel Yorum"],
        ["fx", "1196.13 px", "~2182 px (12mm)", "Mesafe ve eğim eksikliği odak uzaklığını yarı yarıya bastırdı."],
        ["fy", "-1203.66 px", "~ -2182 px", "Elevasyon motor yönü ile piksel satır yönü ters."],
        ["cx, cy", "(1038.8, 1051.2)", "(1024, 1024)", "Optik merkez sensör ortasına başarıyla oturdu."],
        ["k1, k2", "-0.935 / 1.716", "Daha dengeli", "Perspektif gerilmesini dengelemeye çalışırken aşırı şişti."],
        ["p1 (Tanjansiyel)", "<b>-0.1014</b>", "<b>~ 0.0001 (10⁻⁴)</b>", "<b>EN BÜYÜK KANIT:</b> Modelde açı (R) olmadığı için yamukluğu p1 yüklendi!"],
        ["RMS Hatası", "<b>3.6342 px</b>", "< 0.50 px", "Sub-piksel merkez tespiti için yüksek artık hatası."]
    ]
    t_calib = Table(table_calib_data, colWidths=[80, 85, 95, 245])
    t_calib.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), colors.HexColor('#2B6CB0')),
        ('TEXTCOLOR', (0,0), (-1,0), colors.white),
        ('FONTNAME', (0,0), (-1,0), 'Arial-Bold'),
        ('FONTSIZE', (0,0), (-1,-1), 8),
        ('GRID', (0,0), (-1,-1), 0.5, colors.HexColor('#CBD5E0')),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('TOPPADDING', (0,0), (-1,-1), 3.5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 3.5),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, colors.HexColor('#F7FAFC')])
    ]))

    if os.path.exists(img_mesh):
        im_mesh_flow = Image(img_mesh, width=170, height=170)
        side_table = Table([[im_mesh_flow, t_calib]], colWidths=[180, 325])
        side_table.setStyle(TableStyle([
            ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
            ('ALIGN', (0,0), (0,0), 'CENTER'),
        ]))
        story.append(side_table)
        story.append(Paragraph("<b>Şekil 3:</b> 30×30 kalibrasyon sonucunda üretilen kırmızı ızgara. Üst kenarın dar, alt kenarın geniş olduğu (trapezoid/keystone) açıkça görülmektedir.", caption_style))
    else:
        story.append(t_calib)

    # 5. BÖLÜM: YENİ 3B [R | T] KİNEMATİK PROJEKSİYON MODELİ
    story.append(Paragraph("5. Yeni 3B [R | T] Kinematik Projeksiyon Modeli", h1_style))
    model_desc = (
        "Gözlemlenen yamukluğu lens distorsiyonundan tamamen arındırmak ve fiziksel gerçekliği yansıtmak amacıyla "
        "<code>laser_grid_scan.py</code> koduna <b>14 Serbestlik Dereceli</b> yeni projeksiyon hattı entegre edilmiştir:"
    )
    story.append(Paragraph(model_desc, body_style))

    eq_box = [
        [Paragraph("<b>Adım 1: Pinhole 3B Konumu (Gimbal Çerçevesi)</b>", h2_style)],
        [Paragraph("Gimbal motor adımlarından hesaplanan açılarla (azimut θx, elevasyon θy) pinhole'ün uzaydaki konumu milimetre cinsinden üretilir:<br/>"
                   "&nbsp;&nbsp;&nbsp;&nbsp;<b>P_gimbal</b> = D · [tan(θx),  tan(θy),  1]ᵀ &nbsp;&nbsp;&nbsp;&nbsp;<i>(burada D = 80.0 mm nominal çalışma mesafesidir)</i>", body_style)],
        [Paragraph("<b>Adım 2: Kamera Koordinat Sistemine Dönüşüm (Extrinsics)</b>", h2_style)],
        [Paragraph("Kameranın masadaki açısal duruşu (R) ve konumsal kaçıklığı (T) uygulanır:<br/>"
                   "&nbsp;&nbsp;&nbsp;&nbsp;<b>P_cam</b> = R · <b>P_gimbal</b> + T &nbsp;&nbsp;&nbsp;&nbsp;<i>(R: Rodrigues [rx, ry, rz] açıları, T: [tx, ty, tz] ötelemesi)</i>", body_style)],
        [Paragraph("<b>Adım 3: Perspektif Bölme ve Brown-Conrady Distorsiyonu</b>", h2_style)],
        [Paragraph("&nbsp;&nbsp;&nbsp;&nbsp;x' = X_c / Z_c, &nbsp;&nbsp; y' = Y_c / Z_c, &nbsp;&nbsp; r² = x'² + y'²<br/>"
                   "&nbsp;&nbsp;&nbsp;&nbsp;x_dist = x' · (1 + k₁r² + k₂r⁴) + 2p₁x'y' + p₂(r² + 2x'²)<br/>"
                   "&nbsp;&nbsp;&nbsp;&nbsp;y_dist = y' · (1 + k₁r² + k₂r⁴) + p₁(r² + 2y'²) + 2p₂x'y'", body_style)],
        [Paragraph("<b>Adım 4: Piksel İzdüşümü</b>", h2_style)],
        [Paragraph("&nbsp;&nbsp;&nbsp;&nbsp;<b>xu</b> = fx · x_dist + cx, &nbsp;&nbsp;&nbsp;&nbsp; <b>yu</b> = fy · y_dist + cy", body_style)]
    ]
    t_eq = Table(eq_box, colWidths=[505])
    t_eq.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,-1), colors.HexColor('#EDF2F7')),
        ('BOX', (0,0), (-1,-1), 0.5, colors.HexColor('#CBD5E0')),
        ('TOPPADDING', (0,0), (-1,-1), 3),
        ('BOTTOMPADDING', (0,0), (-1,-1), 3),
        ('LEFTPADDING', (0,0), (-1,-1), 10),
        ('RIGHTPADDING', (0,0), (-1,-1), 10),
    ]))
    story.append(t_eq)

    story.append(Spacer(1, 8))

    # 6. BÖLÜM: KAZANIMLAR VE SONUÇ
    story.append(Paragraph("6. Modelin Sağladığı Kazanımlar ve Sonuç", h1_style))
    story.append(Paragraph("1. <b>Mekanik Eğimin Ayrıştırılması:</b> Masadaki bacakların sebep olduğu açısal eğim doğrudan <code>Pitch</code>, <code>Yaw</code> ve <code>Roll</code> olarak derece cinsinden bağımsız raporlanacaktır. Yamukluk distorsiyondan ayrıştırılmıştır.", bullet_style))
    story.append(Paragraph("2. <b>Gerçek Lens Kalitesi:</b> p₁ tanjansiyel distorsiyon yapay -0.10 seviyesinden gerçek lens sınırına (~10⁻⁴) geri dönecektir.", bullet_style))
    story.append(Paragraph("3. <b>Odak Uzaklığı Uyumu:</b> fx ve fy parametreleri Basler 12mm lensin fiziksel piksel karşılığı olan ~2182 px değerine yakınsayacaktır.", bullet_style))
    story.append(Paragraph("4. <b>RMS Hata Düşüşü:</b> Model yetersizliğinden kaynaklanan 3.63 piksel artık hatasının sub-piksel seviyesine (<0.4 - 0.5 px) inmesi sağlanacaktır.", bullet_style))
    story.append(Paragraph("5. <b>İdeal Görselleştirme Güncellemesi:</b> `barrel_pincushion.png` grafiğinde ideal yeşil ızgara da R ve T açıları dahil edilerek çizdirilmekte, böylece saf optik distorsiyon hatasız karşılaştırılabilmektedir.", bullet_style))

    story.append(Spacer(1, 10))
    story.append(Paragraph("<i>Not: Geliştirilen tüm algoritmalar ve matematiksel model güncellemeleri Git deposuna commit edilmiş ve pushlanmıştır. Sistem laboratuvarda yeni test taramalarına hazırdır.</i>", caption_style))

    doc.build(story, canvasmaker=NumberedCanvas)
    print(f"PDF successfully created: {output_path}")

if __name__ == '__main__':
    desktop_pdf = r'C:\Users\user\Desktop\GZGSG_Gunluk_Muhendislik_Raporu_2026-11-05.pdf'
    docs_pdf = r'C:\Users\user\Desktop\gimbal\GZGSG-Distortion-Measurement\docs\GZGSG_Gunluk_Muhendislik_Raporu_2026-11-05.pdf'
    
    build_pdf(desktop_pdf)
    build_pdf(docs_pdf)
